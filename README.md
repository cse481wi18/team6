# CSE 481C
Starter code and samples for CSE 481C at the University of Washington, Winter 2018.

Labs and other documentation are on the **[wiki](https://github.com/cse481wi18/cse481wi18/wiki)**.


# Startup Instructions
## General

## Frontend
Requires: navigation launch and the ActionLibServer responsible for segmenting pointcloud to be running
* `roslaunch recycle_ui classification_server.launch`
* `project && cd recycle-ui/frontend && polymer serve -H 0.0.0.0`

The Polymer command will tell you which port the frontend is running on and you should be able to visit `mifune.cs.washington.edu:PORT`

## Controller
Requires:
* `roslaunch recycle navigation.launch`
* `roslaunch fetch_api move_group.launch`
* Classifier ActionlibServer
* `roslaunch recycle controller.launch`