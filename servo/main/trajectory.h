/* * * * * * * * * * * * * * * * * * * * * * *
 * TRAJECTORY CONTROLLER (Pure C Version)
 * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TRAJECTORY_C_H
#define TRAJECTORY_C_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_timer.h"

// Struct definition
typedef struct {
    int type;          // 0: Position, 1: Velocity
    float threshold;
    float target;
    float curPos;
    float curVel;
    float maxVel;
    float velGoal;
    float acc;
    float dec;
    unsigned long oldTime;
    bool noTasks;
} Trajectory_t;

// Helper: Get millis
static inline unsigned long traj_millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

// Init
static void traj_init(Trajectory_t *traj, float _maxVel, float _acc, float _dec, float _thresh) {
    traj->type = 0;
    traj->target = 0;
    traj->curPos = 0;
    traj->curVel = 0;
    traj->maxVel = _maxVel;
    traj->velGoal = _maxVel;
    traj->acc = _acc;
    if (_dec == -1) traj->dec = _acc;
    else traj->dec = _dec;
    traj->oldTime = traj_millis();
    traj->threshold = _thresh;
    traj->noTasks = true;
}

static void traj_reset(Trajectory_t *traj, float newPos) {
    traj->curPos = newPos;
    traj->curVel = 0;
    traj->target = 0;
    traj->velGoal = traj->maxVel;
    traj->noTasks = true;
    traj->oldTime = traj_millis();
}

static void traj_setTargetPos(Trajectory_t *traj, float _targetPos) {
    traj->target = _targetPos;
    traj->velGoal = traj->maxVel;
    traj->type = 0;
    traj->noTasks = false;
}

static float traj_update_dt(Trajectory_t *traj, float dT) {
    // dT in ms, convert to seconds
    dT /= 1000.0f;

    if (traj->type == 0) {
        float posError = traj->target - traj->curPos;
        if (fabsf(posError) > traj->threshold) {
            bool dir = true;
            if (posError < 0) dir = false;

            float acceleration = traj->acc;
            if ((traj->curVel * traj->curVel / (2 * traj->dec)) >= fabsf(posError)) {
                acceleration = -traj->dec;
            }

            if (dir) traj->curVel += acceleration * dT;
            else traj->curVel -= acceleration * dT;

            // Limit Velocity
            if (traj->curVel > traj->velGoal) traj->curVel = traj->velGoal;
            else if (traj->curVel < -traj->velGoal) traj->curVel = -traj->velGoal;
            
            float dP = traj->curVel * dT;

            if (fabsf(dP) < fabsf(posError)) traj->curPos += dP;
            else traj->curPos = traj->target;

        } else {
            traj->curVel = 0;
            traj->curPos = traj->target;
            traj->noTasks = true;
        }
    } 
    // We only use Position mode for this project, ignoring Velocity mode for brevity/size

    return traj->curPos;
}

static float traj_update(Trajectory_t *traj) {
    unsigned long newTime = traj_millis();
    float dT = (float)(newTime - traj->oldTime);
    traj->oldTime = newTime;
    if (dT > 1000.0f) dT = 1000.0f; // Clamp
    return traj_update_dt(traj, dT);
}

#endif /* TRAJECTORY_C_H */
