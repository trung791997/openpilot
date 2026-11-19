from contextlib import ExitStack
from unittest import mock

import pytest

from opendbc.car.structs import car
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.ioniq5pe_long_profile import candidate as pe_long
from opendbc.car.hyundai.ev9_longitudinal import candidate as ev9_long
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE, intent


