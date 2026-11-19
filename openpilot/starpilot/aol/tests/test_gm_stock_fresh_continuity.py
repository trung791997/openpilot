"""A fresh stock-GM LAT source does not inherit an expired output's PCM-off veto."""
from contextlib import ExitStack
from unittest.mock import patch

import pytest
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_gm_stock_gas_source_gap import params
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import intent

BASE = 10_000_000_000


