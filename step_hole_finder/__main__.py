"""Allow `python -m step_hole_finder -- step.step` style invocation."""

from .find_holes import main

raise SystemExit(main())
