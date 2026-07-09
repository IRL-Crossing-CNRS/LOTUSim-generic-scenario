"""Bundled runtime assets for the ``fault_inspection`` task.

Ships the standalone YOLO corrosion/crack detection server
(``yolo_server_corrosion_crack.py``) and its model weights (``crack.pt``)
*inside the SDK wheel*, so the task can launch the server on whatever machine
runs the agent — host-side or remote — with no path or copy-per-agent juggling.

The heavy ML dependencies the server needs (torch, ultralytics, opencv, flask)
are declared as the ``lotusim_sdk[inspection]`` optional extra, not as core SDK
requirements.
"""
