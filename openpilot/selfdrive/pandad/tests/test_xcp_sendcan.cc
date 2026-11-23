#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <unistd.h>

#include "common/tests/native_test.h"
#include "common/timing.h"
#include "openpilot/cereal/messaging/messaging.h"
#include "selfdrive/pandad/pandad.h"

#ifdef __COMMA_HARDWARE__
std::string real_read_file(const std::string &) asm("__real__ZN4util9read_fileERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE");
std::string wrapped_read_file(const std::string &) asm("__wrap__ZN4util9read_fileERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE");
std::string wrapped_read_file(const std::string &path) {
  if (path == "/sys/firmware/devicetree/base/model") {
    std::string actual = real_read_file(path);
    return actual.empty() ? "comma mici\n" : actual;
  }
  return real_read_file(path);
}
#endif

namespace {

struct RecordedFrame {
  uint8_t bus = 0;
  uint32_t addr = 0;
  bool extended = false;
  std::vector<uint8_t> dat;
};

class RecordingHandle : public PandaCommsHandle {
public:
  std::mutex mtx;
  std::condition_variable cv;
  std::vector<RecordedFrame> sent;

  explicit RecordingHandle(std::string serial) {
    hw_serial = std::move(serial);
  }

  int control_write(uint8_t, uint16_t, uint16_t, unsigned int) override { return 0; }
  int control_read(uint8_t, uint16_t, uint16_t, unsigned char *, uint16_t, unsigned int) override { return -1; }
  int bulk_read(unsigned char, unsigned char *, int, unsigned int) override { return 0; }
  void cleanup() override {}

  int bulk_write(unsigned char endpoint, unsigned char *data, int length, unsigned int) override {
    CHECK(endpoint == 3);
    std::vector<RecordedFrame> batch;
    int pos = 0;
    while (pos < length) {
      CHECK(pos + static_cast<int>(sizeof(can_header)) <= length);
      can_header hdr{};
      std::memcpy(&hdr, data + pos, sizeof(can_header));
      CHECK(hdr.data_len_code < std::size(dlc_to_len));
      uint8_t len = dlc_to_len[hdr.data_len_code];
      int pkt_len = static_cast<int>(sizeof(can_header)) + len;
      CHECK(pos + pkt_len <= length);
      RecordedFrame f;
      f.bus = hdr.bus;
      f.addr = hdr.addr;
      f.extended = hdr.extended != 0;
      f.dat.assign(data + pos + sizeof(can_header), data + pos + pkt_len);
      batch.push_back(std::move(f));
      pos += pkt_len;
    }
    {
      std::lock_guard<std::mutex> lk(mtx);
      sent.insert(sent.end(), batch.begin(), batch.end());
    }
    cv.notify_all();
    return length;
  }

  bool wait_for_count(size_t target, int timeout_ms = 2000) {
    std::unique_lock<std::mutex> lk(mtx);
    return cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] { return sent.size() >= target; });
  }
};

class MockPanda : public Panda {
public:
  MockPanda(std::unique_ptr<RecordingHandle> h, uint32_t offset = 0)
    : Panda(std::move(h), cereal::PandaState::PandaType::RED_PANDA, offset) {}
};

struct PrefixGuard {
  std::string old_prefix;
  std::string shm_dir;
  explicit PrefixGuard(const std::string &prefix) : shm_dir("/dev/shm/msgq_" + prefix) {
    const char *old = std::getenv("OPENPILOT_PREFIX");
    old_prefix = old ? old : "";
    std::error_code ec;
    std::filesystem::create_directories(shm_dir, ec);
    setenv("OPENPILOT_PREFIX", prefix.c_str(), 1);
  }
  ~PrefixGuard() {
    if (old_prefix.empty()) {
      unsetenv("OPENPILOT_PREFIX");
    } else {
      setenv("OPENPILOT_PREFIX", old_prefix.c_str(), 1);
    }
    std::error_code ec;
    std::filesystem::remove_all(shm_dir, ec);
  }
};

void send_can_msg(PubMaster &pm, const char *service, bool is_xcp, uint32_t addr, uint8_t bus, const std::vector<uint8_t> &payload, uint64_t mono_ns) {
  MessageBuilder msg;
  auto evt = msg.initEvent();
  evt.setValid(true);
  evt.setLogMonoTime(mono_ns);
  auto list = is_xcp ? evt.initXcpSendcan(1) : evt.initSendcan(1);
  list[0].setAddress(addr);
  list[0].setSrc(bus);
  list[0].setDat(kj::arrayPtr(payload.data(), payload.size()));
  pm.send(service, msg);
}

void test_dual_threads() {
  PrefixGuard prefix("test-xcp-sendcan-" + std::to_string(getpid()));

  auto handle = std::make_unique<RecordingHandle>("mock-panda");
  auto *rec = handle.get();
  MockPanda panda(std::move(handle), 0);
  std::vector<Panda *> pandas{&panda};

  // Create publishers before starting subscriber threads so msgq shm files exist.
  PubMaster pm({"sendcan", "xcpSendcan"});

  // Phase 1: Live concurrent sendcan + xcpSendcan delivery and stale rejection
  do_exit = false;
  std::thread t_send(can_send_thread, pandas, false);
  std::thread t_xcp(xcp_can_send_thread, pandas, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const std::vector<uint8_t> ctrl_payload{1, 2, 3, 4, 5, 6, 7, 8};
  const std::vector<uint8_t> xcp_payload{0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  send_can_msg(pm, "sendcan", false, 0x1E4, 0, ctrl_payload, nanos_since_boot());
  CHECK(rec->wait_for_count(1));

  send_can_msg(pm, "xcpSendcan", true, 0x1EC00079, 1, xcp_payload, nanos_since_boot());
  CHECK(rec->wait_for_count(2));

  // Concurrent interleaved burst on both topics
  constexpr int kBurst = 20;
  for (int i = 0; i < kBurst; ++i) {
    std::vector<uint8_t> c{static_cast<uint8_t>(i), 1, 2, 3, 4, 5, 6, 7};
    std::vector<uint8_t> x{0xF4, 6, 0, 0, 0x40, 0x03, 0x71, static_cast<uint8_t>(0xB4 + i)};
    send_can_msg(pm, "sendcan", false, 0x1E4, 0, c, nanos_since_boot());
    send_can_msg(pm, "xcpSendcan", true, 0x1EC00079, 1, x, nanos_since_boot());
  }
  CHECK(rec->wait_for_count(2 + 2 * kBurst));

  // Stale xcpSendcan (> 1 s old) must be dropped without sending
  uint64_t now_ns = nanos_since_boot();
  uint64_t stale_ns = now_ns > 2000000000ULL ? (now_ns - 2000000000ULL) : 0ULL;
  send_can_msg(pm, "xcpSendcan", true, 0x1EC00079, 1, {0xFE, 0, 0, 0, 0, 0, 0, 0}, stale_ns);
  // Follow with a fresh sentinel so wait_for_count proves the stale frame was already processed and dropped
  send_can_msg(pm, "xcpSendcan", true, 0x1EC00079, 1, {0xFD, 0, 0, 0, 0, 0, 0, 0}, nanos_since_boot());
  CHECK(rec->wait_for_count(3 + 2 * kBurst));

  do_exit = true;
  t_send.join();
  t_xcp.join();

  {
    std::lock_guard<std::mutex> lk(rec->mtx);
    CHECK(rec->sent.size() == 3 + 2 * kBurst);
    int ctrl_cnt = 0;
    int xcp_cnt = 0;
    for (const auto &f : rec->sent) {
      if (f.addr == 0x1E4 && f.bus == 0) {
        CHECK(!f.extended);
        ++ctrl_cnt;
      } else if (f.addr == 0x1EC00079 && f.bus == 1) {
        CHECK(f.extended);
        CHECK(f.dat.size() == 8);
        CHECK(f.dat[0] != 0xFE);  // stale frame was rejected
        ++xcp_cnt;
      } else {
        CHECK(false);
      }
    }
    CHECK(ctrl_cnt == 1 + kBurst);
    CHECK(xcp_cnt == 2 + kBurst);
    rec->sent.clear();
  }

  // Phase 2: fake_send=true suppresses delivery on xcpSendcan
  do_exit = false;
  std::thread t_xcp_fake(xcp_can_send_thread, pandas, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  send_can_msg(pm, "xcpSendcan", true, 0x1EC00079, 1, xcp_payload, nanos_since_boot());
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  do_exit = true;
  t_xcp_fake.join();

  std::lock_guard<std::mutex> lk(rec->mtx);
  CHECK(rec->sent.empty());
}

}  // namespace

int main() {
  return run_native_test(test_dual_threads);
}
