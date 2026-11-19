from contextlib import ExitStack
from unittest import mock

import pytest

from openpilot.cereal import log
from openpilot.starpilot.aol.tests.test_stock_location_family_intent import stock_family_callers
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE


