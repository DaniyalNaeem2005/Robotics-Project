#include <webots/robot.h>
#include <webots/utils/motion.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#define TIME_STEP 32
#define KICK_THRESHOLD 46
#define DEBUG_CAMERA 1
#define MIN_VALID_BALL_SIZE 10
#define MAX_VALID_BALL_SIZE 100

typedef struct {
    int timestamp;
    int ballSize;
    int ballCenterX;
    int ballCenterY;
    int bluePixels;
    int walkStepsTaken;
    char kickType[50];
    int cameraUsed;
} KickData;

typedef struct {
    int detected;
    int size;
    int centerX;
    int centerY;
    int bluePixels;
    float distanceEstimate;
} BallInfo;
KickData lastKickData;
// Detect blue pixels (of ball)
int isBluePixel(int r, int g, int b) {
    return (b > r + 30 && b > g + 30);}
// Detect blue ball  from camera
BallInfo detectBlueBall(WbDeviceTag camera, int cameraId, const char* cameraName) {
    BallInfo info;
    info.detected = 0;
    info.size = 0;
    info.centerX = -1;
    info.centerY = -1;
    info.bluePixels = 0;
    info.distanceEstimate = 0;  
    const unsigned char *image = wb_camera_get_image(camera);
    if (!image) return info;
    int width = wb_camera_get_width(camera);
    int height = wb_camera_get_height(camera);
    int minX = width, maxX = 0;
    int minY = height, maxY = 0;
    int blueCount = 0;
    
    for (int y = 0; y < height; y += 2) {
        for (int x = 0; x < width; x += 2) {
            int r = wb_camera_image_get_red(image, width, x, y);
            int g = wb_camera_image_get_green(image, width, x, y);
            int b = wb_camera_image_get_blue(image, width, x, y);
            if (isBluePixel(r, g, b)) {
                blueCount++;
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
    }}}
    
    int ballWidth = maxX - minX;
    int ballHeight = maxY - minY;
    int ballSize = (ballWidth + ballHeight) / 2;
    if (blueCount > 10 && ballSize >= MIN_VALID_BALL_SIZE && ballSize <= MAX_VALID_BALL_SIZE) {
        info.detected = 1;
        info.size = ballSize;
        info.centerX = (minX + maxX) / 2;
        info.centerY = (minY + maxY) / 2;
        info.bluePixels = blueCount;
        info.distanceEstimate = 200.0 / ballSize; 
        if (DEBUG_CAMERA) {
            printf("✅ [%s]   BLUE Ball! Size:%dpx X:%d Y:%d Blue:%d Dist:%.1fft\n", 
                   cameraName, info.size, info.centerX, info.centerY, blueCount, info.distanceEstimate);
        }
    }
    return info;
}

// Check both cameras
BallInfo detectBallFromBothCameras(WbDeviceTag camTop, WbDeviceTag camBottom) {
    BallInfo bottomInfo = detectBlueBall(camBottom, 1, "BOTTOM");
    if (bottomInfo.detected) return bottomInfo; 
    BallInfo topInfo = detectBlueBall(camTop, 0, " TOP");
    if (topInfo.detected) return topInfo;
    BallInfo empty;
    empty.detected = 0;
    return empty;
}
// Walk forward until ball reaches target size
int walkForwardOnly(WbMotionRef walk, WbDeviceTag camTop, WbDeviceTag camBottom, int targetSize) {
    int walkSteps = 0;
    int maxWalkSteps = 15;
    while (walkSteps < maxWalkSteps) {
        BallInfo ball = detectBallFromBothCameras(camTop, camBottom);
        if (!ball.detected) {
            printf(" Ball lost! Stopping walk.\n");
            return walkSteps;
        }
        int progress = (ball.size * 100) / targetSize;
        if (progress > 100) progress = 100;
        printf("\n  [Walk %d] Size: %dpx (target: %dpx) X: %d", 
               walkSteps + 1, ball.size, targetSize, ball.centerX);
        printf(" [");
        for (int i = 0; i < 20; i++) {
            printf("%s", i < progress/5 ? "█" : "░");
        }
        printf("] %d%%\n", progress);
        if (ball.size >= targetSize) {
            printf("\n Target size reached! (%dpx)\n", ball.size);
            return walkSteps;
        }
        printf("   Walking forward ...\n");
        wbu_motion_play(walk);
        while (!wbu_motion_is_over(walk)) {
            wb_robot_step(TIME_STEP);
        }
        wbu_motion_stop(walk);
        wb_robot_step(TIME_STEP * 3);
        walkSteps++;
    }
    return walkSteps;
}

// Execute kick motion
void kickWithLeftFoot(WbMotionRef kick)
 {
    printf("\n KICKING \n");
    wbu_motion_play(kick);
    while (!wbu_motion_is_over(kick)) {
        wb_robot_step(TIME_STEP);
    }
    wbu_motion_stop(kick);
    wb_robot_step(TIME_STEP * 2);
    printf(" KICK COMPLETE!\n");
}

int main() {
    wb_robot_init();
    
    printf("\n========================================\n");
    printf("NAO Robot - BLUE Ball Detection\n");
    printf("Target size: %d pixels\n", KICK_THRESHOLD);
    printf("========================================\n");
    // Initialize cameras
    WbDeviceTag camTop = wb_robot_get_device("CameraTop");
    WbDeviceTag camBottom = wb_robot_get_device("CameraBottom");
    if (!camTop || !camBottom) {
        printf("ERROR: Cameras not found!\n");
        return -1;
    }
    wb_camera_enable(camTop, TIME_STEP);
    wb_camera_enable(camBottom, TIME_STEP);
    printf("✓ Cameras enabled\n");
    // Adjust head to look at ball
    WbDeviceTag headPitch = wb_robot_get_device("HeadPitch");
    WbDeviceTag headYaw = wb_robot_get_device("HeadYaw");
    if (headPitch && headYaw) {
        wb_motor_set_position(headYaw, 0);
        wb_motor_set_position(headPitch, 0.40);
        wb_robot_step(TIME_STEP * 10);
        printf("✓ Head adjusted\n");
    }
    
    // Load  motions
    WbMotionRef walk = wbu_motion_new("../../motions/Forwards.motion");
    if (!walk) {
        printf("ERROR: Mssing Forwards.motion file!\n");
        return -1;
    }
    printf("✓ Walk motion loaded\n");
    
    WbMotionRef kick = wbu_motion_new("../../motions/Shoot.motion");
    if (!kick) {
        printf("ERROR: Shoot.motion mnot found!\n");
        return -1;
    }
    printf("✓ Shoot.motion loaded\n");
    
    // Stabilize cameras
    for (int i = 0; i < 10; i++) {
        wb_robot_step(TIME_STEP);
    }
    
    // PHASE 1: FIND BLUE BALL
    printf("PHASE 1: LOCATING BLUE BALL\n");
    
    BallInfo ballInfo;
    int found = 0;
    int searchSteps = 0;
    int maxSearchSteps = 15;
    
    printf("Looking for BLUE ball...\n");
    for (int i = 0; i < 10; i++) {
        ballInfo = detectBallFromBothCameras(camTop, camBottom);
        if (ballInfo.detected) {
            found = 1;
            printf("\n BLUE BALL FOUND! Size: %dpx, X: %d\n", ballInfo.size, ballInfo.centerX);
            break;
        }
        wb_robot_step(TIME_STEP * 5);
    }
    
    if (!found) {
        printf("\n Blue ball not found, walking forward...\n");
    }
    
    while (!found && searchSteps < maxSearchSteps) {
        printf("Searching %d/%d: Walking...\n", searchSteps + 1, maxSearchSteps);
        wbu_motion_play(walk);
        while (!wbu_motion_is_over(walk)) {
            wb_robot_step(TIME_STEP);
        }
        wbu_motion_stop(walk);
        wb_robot_step(TIME_STEP * 5);
        
        ballInfo = detectBallFromBothCameras(camTop, camBottom);
        if (ballInfo.detected) {
            found = 1;
            printf("\n BLUE BALL FOUND! Size: %dpx, X: %d\n", ballInfo.size, ballInfo.centerX);
            break;
        }
        searchSteps++;
    }
    
    if (!found) {
        printf("\n No blue ball found!\n");
        return -1;
    }
    
    // PHASE 2 : WALK FORWARD
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("PHASE 2: WALKING FORWARD\n");
    printf("Target size: %d pixels\n", KICK_THRESHOLD);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    int walkSteps = walkForwardOnly(walk, camTop, camBottom, KICK_THRESHOLD);
    printf("\n✓ Walking complete! Steps taken: %d\n", walkSteps);
    
    // PHASE 3:  KICK
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("PHASE 3: EXECUTING KICK\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    ballInfo = detectBallFromBothCameras(camTop, camBottom);
    if (ballInfo.detected) {
        printf("\nFinal ball: X=%d, Size=%dpx, Dist:%.1fft\n", ballInfo.centerX, ballInfo.size, ballInfo.distanceEstimate);
    }
    
    wbu_motion_stop(walk);
    
    wb_robot_step(TIME_STEP * 3);
    kickWithLeftFoot(kick);
    
    // Save  kick data
    lastKickData.timestamp = (int)time(NULL);
    lastKickData.ballSize = ballInfo.size;
    lastKickData.ballCenterX = ballInfo.centerX;
    lastKickData.ballCenterY = ballInfo.centerY;
    lastKickData.bluePixels = ballInfo.bluePixels;
    lastKickData.walkStepsTaken = walkSteps;
    sprintf(lastKickData.kickType, "BlueBallKick");
    
    FILE *file = fopen("kick_data_log.txt", "a");
    if (file != NULL) {
        fprintf(file, "BLUE_KICK: size=%d, center=(%d,%d), dist=%.1fft, blue=%d, walks=%d\n",
                lastKickData.ballSize, lastKickData.ballCenterX, lastKickData.ballCenterY,
                ballInfo.distanceEstimate, lastKickData.bluePixels, lastKickData.walkStepsTaken);
        fclose(file);
    }
    
    printf("          🎉 SUCCESS! 🎉\n");
    printf("  Final size: %dpx, Distance: %.1fft, Walks: %d\n", ballInfo.size, ballInfo.distanceEstimate, walkSteps);
    printf("========================================\n");
    
    wbu_motion_delete(walk);
    
    wbu_motion_delete(kick);
    wb_robot_cleanup();
    return 0;
}