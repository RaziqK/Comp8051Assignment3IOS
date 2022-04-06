//
//  Copyright © Borna Noureddin. All rights reserved.
//

#ifndef MyGLGame_CBox2D_h
#define MyGLGame_CBox2D_h

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>


// Set up brick and ball physics parameters here:
//   position, width+height (or radius), velocity,
//   and how long to wait before dropping brick

#define BRICK_POS_X            400
#define BRICK_POS_Y            0
#define BRICK_WIDTH            100.0f
#define BRICK_HEIGHT        10.0f
#define BRICK_WAIT            1.5f

#define BRICK_CPU_POS_X            400
#define BRICK_CPU_POS_Y            550
#define BRICK_CPU_WIDTH            100.0f
#define BRICK_CPU_HEIGHT        10.0f
#define BRICK_CPU_WAIT            1.5f

#define BALL_POS_X            400
#define BALL_POS_Y            300
#define BALL_RADIUS            15.0f
#define BALL_VELOCITY        100000.0f
#define BALL_SPHERE_SEGS    128


@interface CBox2D : NSObject

@property int playerScore;
@property int CPUScore;

-(void) HelloWorld; // Basic Hello World! example from Box2D

-(void) LaunchBall;                 // launch the ball
-(void) Update:(float)elapsedTime;  // update the Box2D engine
-(void) RegisterHit; // Register when the ball hits the brick
-(void) RegisterHitCPU; // When Ball hits CPU Brick
-(void) movePlayerWall:(float)transY;
-(void) PlayerScored:(float)playerScore;
-(void *)GetObjectPositions;        // Get the positions of the ball and brick

@end

#endif
