import openpilot.starpilot.system.the_galaxy.the_galaxy as galaxy


def test_webhook_sends_every_image_under_its_own_field(mocker, tmp_path):
  # Discord kept only wide.jpg when both images were posted under the same "file" field
  paths = []
  for name in ("wide.jpg", "driver.jpg"):
    path = tmp_path / name
    path.write_bytes(b"\xff\xd8jpeg")
    paths.append(str(path))

  params = mocker.MagicMock()
  params.get.side_effect = lambda key, **kw: "https://discord.example/webhook" if key == "SentryModeWebhook" else None
  mocker.patch.object(galaxy, "params", params)
  mocker.patch.object(galaxy, "_sentry_notification_channels", return_value={"webhook": True})
  mocker.patch.object(galaxy, "_dispatch_sentry_push")
  post = mocker.patch.object(galaxy.requests, "post")

  galaxy._dispatch_sentry_event({"eventId": "1", "kind": "alarm", "message": "m", "imagePaths": paths}, bypass_rate_limit=True)

  files = post.call_args.kwargs["files"]
  assert [field for field, _ in files] == ["files[0]", "files[1]"]
  assert [f[0] for _, f in files] == ["wide.jpg", "driver.jpg"]
