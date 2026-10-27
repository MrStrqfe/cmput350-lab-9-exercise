#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // TODO: (HINTS/outline)
        // collect statistics on the player position and velocity in (normalized, if you like)
        // window coordinates. Compute lookAhead distance in world coords, adjusting for
        // if the player is moving left or right.
        // Now update lookAhead state using an EMA over its previous state and the lookAhead world
        // vector. Then determine if the player is in the dead zone, soft rejection zone, or hard
        // rejection zone. For each, perform EMA on the delta of the camera displacement (ignoring
        // lookahead) appropriately. Next,
        // set the camera center by offsetting the cameraUnadjustedCenter, combining lookAhead.
        // Finally update cameraUnadjustedCenter, but without lookahead.
        
        // Measure the player against the camera WITHOUT lookahead, so the zones
        // don't react to the lookahead offset itself.
        camera.setCameraCenter(cameraUnadjustedCenter);
        sf::Vector2f playerPos = camera.worldToWindowPercent(trackedPosWorld);
        sf::Vector2f playerVel = camera.worldToWindowPercentDist(trackedVelocityWorld);

        // Get lookAhead as a world coords
        float lookAheadDist = camera.windowPercentToWorldDist({lookAhead, 0.0f}).x;
        
        // Point the lookAhead based on which way the player is moving
        float target = 0.0f;
        // std::cout << lookAheadWorld.x << ", " << lookAheadWorld.y << "\n";
        if (playerVel.x < 0.0f) {
            target = -lookAheadDist;
            //camera.translateWindowPercent({cameraUnadjustedCenter.x + lookAheadWorld.x, 0.0f});
        }
        else if (playerVel.x > 0.0f) {
            target = lookAheadDist;
        }

        lookaheadState += alpha * (target - lookaheadState);

        // Step 4: dead / soft / hard zone logic (window percent, no lookahead)
        float shiftPercent = 0.0f;  // how far to move the camera this frame
        if (!deadZoneX.contains(playerPos.x)) {
            // Goal: move the camera so the player sits on the dead zone edge
            float deadEdge = 0.0f;
            if (playerPos.x > deadZoneX.max) { 
                deadEdge = deadZoneX.max;
            }
            else {
                deadEdge = deadZoneX.min;
            }

            if (softZoneX.contains(playerPos.x)) {
                // Soft zone: close only alpha of the gap each frame
                shiftPercent = alpha * (playerPos.x - deadEdge);
            } else {
                // Hard zone: snap the player back to the soft zone edge right away,
                // then ease the rest of the way like the soft zone
                float softEdge = 0.0f;
                if (playerPos.x > softZoneX.max) { 
                    softEdge = softZoneX.max; 
                } 
                else { 
                    softEdge = softZoneX.min;
                }
                float hardShift = playerPos.x - softEdge;
                float softShift = softEdge - deadEdge;
                shiftPercent = hardShift + alpha * softShift;
            }
        }
        sf::Vector2f newUnadjustedCenter =
            cameraUnadjustedCenter + camera.windowPercentToWorldDist({shiftPercent, 0.0f});

        // Step 5: real camera position = unadjusted center + lookahead
        camera.setCameraCenter(newUnadjustedCenter + sf::Vector2f{lookaheadState, 0.0f});

        // Step 6: remember the center without lookahead for next frame
        cameraUnadjustedCenter = newUnadjustedCenter;
    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif // DYNAMIC_CAMERA_H
