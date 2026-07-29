# PathPlanner archive

Paths moved out of `src/main/deploy/pathplanner/paths/` because no `.auto`
referenced them. They are kept here so nothing is lost, but PathPlanner does
not see them, they are not deployed to the RoboRIO, and they no longer clutter
the path list when you are building autos on the fly.

To bring one back, move the `.path` file into
`src/main/deploy/pathplanner/paths/` and restart PathPlanner.

Re-run `python3 tools/pathplanner_audit.py --orphans` to find newly orphaned
paths after you delete or edit autos.
