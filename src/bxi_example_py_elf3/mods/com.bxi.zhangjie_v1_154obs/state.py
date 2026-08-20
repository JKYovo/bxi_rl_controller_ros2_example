from bxi_example_py_elf3.policies import DanceMotionPolicyIsaaclab
from bxi_example_py_elf3.framework.mod_api import MotionReplayState, ResourceHandle


class ZhangjieV1State(MotionReplayState[DanceMotionPolicyIsaaclab]):
    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[DanceMotionPolicyIsaaclab],
    ) -> None:
        super().__init__(
            name,
            state_id,
            policy,
            finish_state="com.bxi.basic_actions/normal",
            finish_trigger="zhangjie_v1_154obs_finished",
            end_frame_trim=0,
            end_transition={
                "profile": "dual_running_blend",
                "duration": 1.0,
                "curve": "smootherstep",
                "sample_from": True,
            },
        )
