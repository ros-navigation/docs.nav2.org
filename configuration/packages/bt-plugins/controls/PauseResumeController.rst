.. _bt_pause_resume_controller_control:

PauseResumeController
===================

Controlled through service calls to pause and resume the execution of the tree It has one mandatory child for the RESUMED, and three optional for the PAUSED state, the ON_PAUSE event and the ON_RESUME event.
It has two input ports:

- pause_service_name: name of the service to pause
- resume_service_name: name of the service to resume

The controller starts in RESUMED state, and ticks it until it returns success.
When the pause service is called, ON_PAUSE is ticked until completion, then the controller switches to PAUSED state.
When the resume service is called, ON_RESUME is ticked until completion, then the controller switches back to RESUMED state.

The controller only returns success when the RESUMED child returns success.
The controller returns failure if any child returns failure.
In any other case, it returns running.

Example
-------

.. code-block:: xml

    <PauseResumeController pause_service_name="/pause" resume_service_name="/resume">
        <!-- RESUMED branch (mandatory) -->

        <!-- PAUSED branch (optional) -->

        <!-- ON_PAUSE branch (optional) -->

        <!-- ON_RESUME branch (optional) -->
    </PauseResumeController>

When the ON_PAUSE and ON_RESUME branches fail, the controller will return failure, halt, and the state will be reset to RESUMED. It might be desirable to retry the transition a few times before failing for real, which functionality is not built in the controller node, but is easily achievable by adding a retry node in the BT:

.. code-block:: xml

    <PauseResumeController pause_service_name="/pause" resume_service_name="/resume">
        <!-- RESUMED branch -->

        <!-- PAUSED branch -->

        <RetryUntilSuccessful num_attempts="3">
            <!-- ON_PAUSE branch -->
        </RetryUntilSuccessful>

        <RetryUntilSuccessful num_attempts="3">
            <!-- ON_RESUME branch -->
        </RetryUntilSuccessful>
    </PauseResumeController>
