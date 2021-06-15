#pragma once

extern const int planner_movesPlanned();
extern void planner_zeroSpeeds();
extern void wait_for_empty_segment_buffer();
extern bool planner_segmentBufferFull();

extern void planner_addSegment(const float *const target_position, float fr_units_s, float millimeters);
extern void planner_bufferLine(float *pos, float new_feed_rate_units);
void planner_bufferArc(float cx, float cy, float *destination, char clockwise, float new_feed_rate);