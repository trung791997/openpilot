import json
from pathlib import Path


ASSETS = Path(__file__).parent / 'assets/navigation'
MANIFEST = {row['file']: row for row in json.loads(Path(__file__).with_name('navigation-assets.json').read_text())['files']}


def icon_name(maneuver_type: str, modifier: str) -> str:
  # Mapbox's 'roundabout turn' (a turn at a small roundabout) is drawn as the plain turn for its modifier, and the exits
  # of a roundabout or rotary as the roundabout; neither had a file of its own, so both got the straight arrow.
  kind = {'rotary': 'roundabout', 'exit roundabout': 'roundabout', 'exit rotary': 'roundabout', 'roundabout turn': 'turn',
          'new name': 'turn', 'continue': 'turn'}.get(maneuver_type, maneuver_type).replace(' ', '_')
  suffix = {'slightLeft': 'slight_left', 'slightRight': 'slight_right', 'sharpLeft': 'sharp_left',
            'sharpRight': 'sharp_right'}.get(modifier, modifier).replace(' ', '_')
  candidate = 'direction_uturn.png' if suffix == 'uturn' else f'direction_{kind}{"_" + suffix if suffix else ""}.png'
  return candidate if candidate in MANIFEST else 'direction_turn_straight.png'
