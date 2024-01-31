uint32 sender_agent_id
byte[] serialized_map # Serialized map containing the new keyframes
byte[16] reference_key_frame_uuid # The existing keyframe that is used as a reference to weld on new keyframes
byte[16] next_reference_key_frame_uuid # The new keyframe which will be used as the next reference keyframe