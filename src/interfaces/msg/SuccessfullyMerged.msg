std_msgs/Header header
uint32 sender_agent_id
uint32 receiver_agent_id
bool successfully_merged
bool implicit_merge # An implicit merge is when we are merging simply because the lead agent in our group merged with their lead agent (lead agent is the one with the smallest agentId)
Uuid[] merged_key_frame_uuids # Used so the receiver can try merge by looking at these keyframes
Uuid[] all_key_frames_in_map # Used so the receiver knows which keyframes have already been sent, so we dont resend them