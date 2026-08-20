from bxi_example_py_elf3.policies import DanceMotionPolicyGravityIsaaclabV3
from bxi_example_py_elf3.framework.mod_api import MotionReplayState, ResourceHandle


class LmmTurnV3State(MotionReplayState[DanceMotionPolicyGravityIsaaclabV3]):
    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[DanceMotionPolicyGravityIsaaclabV3],
    ) -> None:
        super().__init__(
            name,
            state_id,
            policy,
            finish_state="com.bxi.basic_actions/normal",
            finish_trigger="lmm_turn_v3_finished",
            end_frame_trim=0,
            end_transition={
                "profile": "dual_running_blend",
                "duration": 1.0,
                "curve": "smootherstep",
                "sample_from": True,
            },
        )
