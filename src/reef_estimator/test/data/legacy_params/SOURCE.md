# Verbatim legacy parameter files

Byte-identical copies of `params/*.yaml` from UF REEF `reef_estimator` master
`e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f` (MIT License, © 2020 University of
Florida REEF Autonomous Vehicles Lab). They are test inputs only: they show how
ROS 2 parses the original files, and they are the reference for the
converted `config/estimator_master.yaml`. Do not edit them.

| File | Git blob at `e4179f48` |
|---|---|
| `xy_est_params.yaml` | `6d113d3c848f5fc4c5bcdf7f8b84b736c9671dc5` |
| `z_est_params.yaml` | `2ab5bf6bb3ae6590374d54f93f9b78f2947d5bb0` |
| `basic_params.yaml` | `7aece7de76b94f33758309d0712da0bf19432690` |

`test/test_legacy_params.py` checks these blob IDs.
