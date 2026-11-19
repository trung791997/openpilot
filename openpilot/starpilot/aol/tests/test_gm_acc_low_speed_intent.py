"""Cruise's engagement floor cannot clear independent GM steering intent."""
from contextlib import ExitStack

import pytest
from opendbc.car.gm.tests.test_ascm_intercept import params
from opendbc.car.gm.values import CAR
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_gm as gm_fixture, test_ioniq_availability as fixture
from openpilot.starpilot.aol.wire import IntentState, encode_intent
from openpilot.starpilot.audio.tests.test_axis_alerts import AudioFixture, AudibleAlert


