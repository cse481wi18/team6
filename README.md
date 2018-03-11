# CSE 481C
Starter code and samples for CSE 481C at the University of Washington, Winter 2018.

Labs and other documentation are on the **[wiki](https://github.com/cse481wi18/cse481wi18/wiki)**.


# Startup Instructions
## General
Run all of these on `astro`

Libraries
* `roslaunch recycle navigation.launch`
* `roslaunch fetch_api move_group.launch`

Recycle Backend
* `rosrun recycle logger_server.launch` (`<node>`)
* `roslaunch recycle classifier.launch`
* `roslaunch recycle controller.launch`
* `roslaunch recycle_ui classification_server.launch`


## Frontend
Run this on `mifune.cs.washington.edu`. Make sure to make sure all the others are running on `astro` and run `setrobot astro` before running the following commands.
* `roslaunch recycle_ui web-stuff.launch`
* `project && cd recycle-ui/frontend && polymer serve -H 0.0.0.0`

The Polymer command will tell you which port the frontend is running on and you should be able to visit `mifune.cs.washington.edu:PORT`

## Controller
Requires:
* `roslaunch recycle navigation.launch`
* `roslaunch fetch_api move_group.launch`
* Classifier ActionlibServer
* `roslaunch recycle controller.launch`

## Classifier
* Logger
* `roslaunch recycle classifier.launch`
