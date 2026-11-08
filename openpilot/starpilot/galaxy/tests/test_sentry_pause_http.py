"""Pausing/resuming Sentry over the Galaxy API requires a current authenticated session."""
import http.client
import json
import os
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from openpilot.common.params import Params
from openpilot.starpilot.galaxy.access import GalaxyAccessOwner
from openpilot.starpilot.galaxy.server import make_server
from openpilot.starpilot.sentry_mode.pause import MAX_SECONDS, is_paused


class SentryPauseHTTPTests(unittest.TestCase):
  def setUp(self):
    self.directory = tempfile.TemporaryDirectory()
    self.addCleanup(self.directory.cleanup)
    self.params_directory = tempfile.TemporaryDirectory()
    self.addCleanup(self.params_directory.cleanup)
    self.environment = patch.dict(os.environ, PARAMS_ROOT=self.params_directory.name, OPENPILOT_PREFIX="sentrypausehttp")
    self.environment.start()
    self.addCleanup(self.environment.stop)
    self.params = Params()
    from pathlib import Path
    root = Path(self.directory.name).resolve()
    self.server = make_server(port=0, owner=GalaxyAccessOwner(root / 'access'))
    self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    self.thread.start()
    self.addCleanup(self._stop_server)

  def _stop_server(self):
    self.server.shutdown()
    self.thread.join(timeout=2)
    self.server.server_close()

  def request(self, method, payload=None, cookie=''):
    connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
    headers = {'Cookie': cookie, 'Origin': f'http://127.0.0.1:{self.server.server_port}', 'Content-Type': 'application/json'}
    connection.request(method, '/api/sentry/pause', json.dumps(payload) if payload is not None else None, headers)
    reply = connection.getresponse()
    result = reply.status, json.loads(reply.read()), dict(reply.getheaders())
    connection.close()
    return result

  def session_cookie(self):
    connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
    connection.request('GET', '/api/auth/session', None, {'Origin': f'http://127.0.0.1:{self.server.server_port}'})
    reply = connection.getresponse()
    cookie = dict(reply.getheaders())['Set-Cookie'].split(';', 1)[0]
    reply.read()
    connection.close()
    return cookie

  def test_requires_session_for_get_and_post(self):
    self.assertEqual(self.request('GET')[0], 401)
    self.assertEqual(self.request('POST', {'until': int(time.time()) + 60})[0], 401)

  def test_set_and_clear_round_trip(self):
    cookie = self.session_cookie()
    code, result, _ = self.request('GET', cookie=cookie)
    self.assertEqual((code, result), (200, {'pausedUntil': None}))

    until = int(time.time()) + 1800
    code, result, _ = self.request('POST', {'until': until}, cookie)
    self.assertEqual((code, result), (200, {'pausedUntil': until}))
    self.assertTrue(is_paused(self.params, until - 1))

    code, result, _ = self.request('POST', {'action': 'clear'}, cookie)
    self.assertEqual((code, result), (200, {'pausedUntil': None}))
    self.assertFalse(is_paused(self.params, until - 1))

  def test_past_time_and_too_far_out_are_rejected(self):
    cookie = self.session_cookie()
    code, result, _ = self.request('POST', {'until': int(time.time()) - 5}, cookie)
    self.assertEqual(code, 400)
    self.assertIn('error', result)

    code, result, _ = self.request('POST', {'until': int(time.time()) + MAX_SECONDS + 100}, cookie)
    self.assertEqual(code, 400)
    self.assertIn('error', result)

  def test_malformed_payload_is_rejected(self):
    cookie = self.session_cookie()
    self.assertEqual(self.request('POST', {}, cookie)[0], 400)
    self.assertEqual(self.request('POST', {'until': 'soon'}, cookie)[0], 400)
    self.assertEqual(self.request('POST', {'action': 'clear', 'until': 1}, cookie)[0], 400)


if __name__ == "__main__":
  unittest.main()
