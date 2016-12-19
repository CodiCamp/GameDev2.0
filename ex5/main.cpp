#include <irrlicht.h>
#include <irrKlang.h>
#include <ozcollide\ozcollide.h>
#include <ozcollide\box.h>
#include <ozcollide\sphere.h>




#include "MyEventReceiver.h"
//#include <cImage.h>
using namespace std;

using namespace irr;
using namespace core;
using namespace video;
using namespace irrklang;
using namespace ozcollide;

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#pragma comment(lib, "irrKlang.lib") // link with irrKlang.dll
#endif


enum states
{
	STATE_PATROLLING,
	STATE_FIRING,
	STATE_MOVING

} currentState;

double seeDistance = 300.0f;

bool seePlayer(Box *enemyShip, Box *playerShip)
{
	double distx = enemyShip->center.x - playerShip->center.x;
	double disty = enemyShip->center.y - playerShip->center.y;
	double dist = sqrt(distx*distx + disty*disty);

	if (dist <= seeDistance)
		return true;
	return false;
}
f32 angleToFacePlayer(Box* enemyShip, Box* playerShip)
{
	f32 i = 0;
	double distx = playerShip->center.x - enemyShip->center.x;
	double disty = playerShip->center.y - enemyShip->center.y;
	i = acos(disty / (sqrt(distx*distx + disty*disty))) * 180 / PI;
	if (distx > 0)
		return i;
	if (distx < 0)
		return -i + 360;
}
bool moveToPosition(double speed, position2df* enemyPosition, position2df lastPosition)
{
	double diffX = lastPosition.X - enemyPosition->X;
	double diffY = lastPosition.Y - enemyPosition->Y;

	enemyPosition->X += diffX*speed;
	enemyPosition->Y += diffY*speed;

	if ((diffX >= 10 || diffX <= -10) && (diffY >= 10 || diffY <= -10))
		return true;
	return false;
}
void drawCollision(Box* box, IVideoDriver* driver)
{
	driver->draw2DLine(vector2d<s32>(box->getPoint(0).x, box->getPoint(0).y), vector2d<s32>(box->getPoint(1).x, box->getPoint(1).y), SColor(255, 255, 0, 0));
	driver->draw2DLine(vector2d<s32>(box->getPoint(1).x, box->getPoint(1).y), vector2d<s32>(box->getPoint(3).x, box->getPoint(3).y), SColor(255, 255, 0, 0));
	driver->draw2DLine(vector2d<s32>(box->getPoint(2).x, box->getPoint(2).y), vector2d<s32>(box->getPoint(3).x, box->getPoint(3).y), SColor(255, 255, 0, 0));
	driver->draw2DLine(vector2d<s32>(box->getPoint(0).x, box->getPoint(0).y), vector2d<s32>(box->getPoint(6).x, box->getPoint(6).y), SColor(255, 255, 0, 0));
	box->~Box();
}
void draw2DImageWithRot(IVideoDriver *driver,ITexture* texture,rect<irr::s32> sourceRect, position2d<irr::s32> position, position2d<irr::s32> rotationPoint,f32 rotation,vector2df scale, bool useAlphaChannel, SColor color) {

	// Store and clear the projection matrix
	irr::core::matrix4 oldProjMat = driver->getTransform(irr::video::ETS_PROJECTION);
	driver->setTransform(irr::video::ETS_PROJECTION, irr::core::matrix4());

	// Store and clear the view matrix
	irr::core::matrix4 oldViewMat = driver->getTransform(irr::video::ETS_VIEW);
	driver->setTransform(irr::video::ETS_VIEW, irr::core::matrix4());

	// Store and clear the world matrix
	irr::core::matrix4 oldWorldMat = driver->getTransform(irr::video::ETS_WORLD);
	driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());

	// Find horizontal and vertical axes after rotation
	irr::f32 c = cos(-rotation*irr::core::DEGTORAD);
	irr::f32 s = sin(-rotation*irr::core::DEGTORAD);
	irr::core::vector2df horizontalAxis(c, s);
	irr::core::vector2df verticalAxis(s, -c);

	// First, we'll find the offset of the center and then where the center would be after rotation
	irr::core::vector2df centerOffset(position.X + sourceRect.getWidth() / 2.0f*scale.X - rotationPoint.X, position.Y + sourceRect.getHeight() / 2.0f*scale.Y - rotationPoint.Y);
	irr::core::vector2df center = centerOffset.X*horizontalAxis - centerOffset.Y*verticalAxis;
	center.X += rotationPoint.X;
	center.Y += rotationPoint.Y;

	// Now find the corners based off the center
	irr::core::vector2df cornerOffset(sourceRect.getWidth()*scale.X / 2.0f, sourceRect.getHeight()*scale.Y / 2.0f);
	verticalAxis *= cornerOffset.Y;
	horizontalAxis *= cornerOffset.X;
	irr::core::vector2df corner[4];
	corner[0] = center + verticalAxis - horizontalAxis;
	corner[1] = center + verticalAxis + horizontalAxis;
	corner[2] = center - verticalAxis - horizontalAxis;
	corner[3] = center - verticalAxis + horizontalAxis;

	// Find the uv coordinates of the sourceRect
	irr::core::vector2df textureSize(texture->getSize().Width, texture->getSize().Height);
	irr::core::vector2df uvCorner[4];
	uvCorner[0] = irr::core::vector2df(sourceRect.UpperLeftCorner.X, sourceRect.UpperLeftCorner.Y);
	uvCorner[1] = irr::core::vector2df(sourceRect.LowerRightCorner.X, sourceRect.UpperLeftCorner.Y);
	uvCorner[2] = irr::core::vector2df(sourceRect.UpperLeftCorner.X, sourceRect.LowerRightCorner.Y);
	uvCorner[3] = irr::core::vector2df(sourceRect.LowerRightCorner.X, sourceRect.LowerRightCorner.Y);
	for (irr::s32 i = 0; i < 4; i++)
		uvCorner[i] /= textureSize;

	// Vertices for the image
	irr::video::S3DVertex vertices[4];
	irr::u16 indices[6] = { 0, 1, 2, 3, 2, 1 };

	// Convert pixels to world coordinates
	irr::core::vector2df screenSize(driver->getViewPort().getWidth(), driver->getViewPort().getHeight());
	for (irr::s32 i = 0; i < 4; i++) {
		vertices[i].Pos = irr::core::vector3df(((corner[i].X / screenSize.X) - 0.5f)*2.0f, ((corner[i].Y / screenSize.Y) - 0.5f)*-2.0f, 1);
		vertices[i].TCoords = uvCorner[i];
		vertices[i].Color = color;
	}

	// Create the material
	// IMPORTANT: For irrlicht 1.8 and above you MUST ADD THIS LINE:
	irr::video::SMaterial material;
	material.BlendOperation = irr::video::EBO_ADD;
	material.Lighting = false;
	material.ZWriteEnable = false;
	material.ZBuffer = false;
	material.BackfaceCulling = false;
	material.TextureLayer[0].Texture = texture;
	material.MaterialTypeParam = irr::video::pack_textureBlendFunc(irr::video::EBF_SRC_ALPHA, irr::video::EBF_ONE_MINUS_SRC_ALPHA, irr::video::EMFN_MODULATE_1X, irr::video::EAS_TEXTURE | irr::video::EAS_VERTEX_COLOR);
	//material.BlendOperation = irr::video::EBO_ADD;
	if (useAlphaChannel)
		material.MaterialType = irr::video::EMT_ONETEXTURE_BLEND;
	else
		material.MaterialType = irr::video::EMT_SOLID;

	driver->setMaterial(material);
	driver->drawIndexedTriangleList(&vertices[0], 4, &indices[0], 2);

	// Restore projection, world, and view matrices
	driver->setTransform(irr::video::ETS_PROJECTION, oldProjMat);
	driver->setTransform(irr::video::ETS_VIEW, oldViewMat);
	driver->setTransform(irr::video::ETS_WORLD, oldWorldMat);
}
int main()
{
	f32 M_PI = 3.14159265359;
	s32 rocketAnimFrameSizeW = 55;
	s32 rocketAnimFrameSizeH = 83;

	Box* rocketCollision = new Box();
	Box* enemyCollision = new Box();
	
	//create an instance of the event receiver
	MyEventReceiver receiver;

	// create device
	IrrlichtDevice *device = createDevice(video::EDT_DIRECT3D9,
		core::dimension2d<u32>(800, 600), 16, false, false, false, &receiver);

	if (device == 0)
		return 1; // could not create selected driver.

	ISoundEngine* soundEngine = createIrrKlangDevice();
	if (!soundEngine)
		return 0; // error starting up the engine


	video::IVideoDriver* driver = device->getVideoDriver();
//	cImage* rocketRot = new cImage();
	//add scene manager
	scene::ISceneManager* smgr = device->getSceneManager();
	
	video::ITexture* rocket = driver->getTexture("./media/Rocket_spritesheet.png");
	//driver->makeColorKeyTexture(image, position2d<s32>(0, 0));

	video::ITexture* bgrnd = driver->getTexture("./media/Background_Purple_Space-800x600.jpg");
	//driver->makeColorKeyTexture(bgrnd, position2d<s32>(0, 0));
	
	video::ITexture* sun = driver->getTexture("./media/sunAnim.PNG");
	driver->makeColorKeyTexture(sun, position2d<s32>(0, 0));

	video::ITexture* enemy = driver->getTexture("./media/YWing.png");
	


	int currentColumn = 0;
	int row = 0;
	int LastFps = -1;

	u32 lastTime = device->getTimer()->getTime();
	u32 lastAnimationFrame = device->getTimer()->getTime();
	u32 lastSunFrame = device->getTimer()->getTime();
	//"+" is CCW, "-" is CW
	f32 rotationAngle = 0;
	
	int previousFrame = 0;

	int sunFrame = 0;

	f32 MOVEMENT_SPEED = 0.2f;
	f32 SPEED_X = 0.0f;
	f32 SPEED_Y = 0.0f;
	f32 acceeration = 0.0005f;
	
	position2df RocketPosition(300,300);
	position2df lastPosition;
	position2df *EnemyPosition = new position2df(20, 20);

	bool fire = false;

	currentState = states::STATE_PATROLLING;



	while (device->run() && !receiver.IsKeyDown(irr::KEY_ESCAPE))
	{
		const u32 currentTime = device->getTimer()->getTime();
		//u32 Dt = currentTime - lastTime;

		const f32 Dt= (f32)(currentTime - lastTime); // Time in seconds
		lastTime = currentTime;

		/*if (rotationAngle >= 360)
			rotationAngle = 0;
		if (rotationAngle < 0)
			rotationAngle = 360 - rotationAngle;*/

		//compute how much time has passed since previous frame. If it is > the time between animation frames, curentframe++
		/* Check if keys W, S, A or D are being held down, and move the
		sphere node around respectively. */
	

		const f32 r = (f32)sqrt(RocketPosition.X*RocketPosition.X + RocketPosition.Y*RocketPosition.Y);
		f32 normR = (RocketPosition.X*RocketPosition.X + RocketPosition.Y*RocketPosition.Y);
		if (receiver.IsKeyDown(irr::KEY_KEY_W))
		{
			RocketPosition.X -=	 Dt* f32(sinf(rotationAngle* M_PI / 180.0f));//!!!IN RADIANS
			RocketPosition.Y -=  Dt* f32(cosf(rotationAngle* M_PI / 180.0f));//!!!IN RADIANS
			if (!soundEngine->isCurrentlyPlaying("./media/explosion.wav"))//check if the sound is not currently playing to avoid sound overlap
				soundEngine->play2D("./media/explosion.wav", false);
		}
		if (receiver.IsKeyDown(irr::KEY_KEY_S))
		{
			//RocketPosition.Y += (MOVEMENT_SPEED * Dt);
		}
		if (receiver.IsKeyDown(irr::KEY_KEY_A))
		{
			//RocketPosition.X -= MOVEMENT_SPEED * Dt;
			rotationAngle += 0.1f * Dt;
		}
		if (receiver.IsKeyDown(irr::KEY_KEY_D))
		{
			//RocketPosition.X += MOVEMENT_SPEED * Dt;
			rotationAngle -= 0.1f * Dt;
		}
		
		rocketCollision->setFromPoints(Vec3f(RocketPosition.X, RocketPosition.Y, 0),
			Vec3f(RocketPosition.X + rocketAnimFrameSizeW, RocketPosition.Y + rocketAnimFrameSizeH, 0));

		enemyCollision->setFromPoints(Vec3f(EnemyPosition->X, EnemyPosition->Y, 0),
			Vec3f(EnemyPosition->X + 128.0f, EnemyPosition->Y + 128.0f, 0));
		
		if (currentTime - lastAnimationFrame >= (1000/60))
		{
			currentColumn++;
			lastAnimationFrame = currentTime;

		}
		if (currentColumn >= 10)
		{
			currentColumn = 0;
			row++;
		}
		if (row >= 6) row = 0;

		//sun animation
		if (currentTime - lastSunFrame >= (1000 / 5))
		{
			sunFrame++;
			lastSunFrame = currentTime;
		}
		if (sunFrame >= 5)
		{
			sunFrame = 0;
		}
	
		/* Check if keys W, S, A or D are being held down, and move the
		sphere node around respectively. */
		
		switch (currentState)
		{
		case STATE_PATROLLING:
			if (seePlayer(enemyCollision, rocketCollision))
			{
				currentState = STATE_FIRING;
			}break;
		case STATE_FIRING:
			if (!seePlayer(enemyCollision, rocketCollision))
			{
				fire = false;
				lastPosition = RocketPosition;
				currentState = STATE_MOVING;
				//TODO function to move to last known player position
			}
			else
			{
				fire = true;
			}break;
		case STATE_MOVING:
		{
			if (seePlayer(enemyCollision, rocketCollision))
				{
					currentState = STATE_FIRING;
					fire = true;
				}

			else
			{
				fire = false;
				if (!moveToPosition(0.0005*Dt, EnemyPosition, lastPosition))
				{
					currentState = STATE_PATROLLING;
				}
			}
		}
			break;

		}

		driver->beginScene(true, true, SColor(255, 113, 113, 133));

		//smgr->drawAll(); // draw the 3d scene
		
			driver->draw2DImage(bgrnd, position2d<s32>(0, 0),
				rect<s32>(0, 0,
				800, 600), 0,
				SColor(255, 255, 255, 255),true);

			driver->draw2DImage(sun, position2d<s32>(300, 200),
				rect<s32>(sunFrame * 200, 0,
				(sunFrame + 1) * 200, 200), 0,
				SColor(255, 255, 255, 255), true);
			/*driver->draw2DImage(enemy,
				position2d<s32>((s32)EnemyPosition->X, (s32)EnemyPosition->Y),
				rect<s32>(0, 0,
				128.0f, 128.0f), 0,
				SColor(255, 255, 255, 255), true);*/
			draw2DImageWithRot(driver, enemy, rect<s32>(0, 0, 128, 128), 
				position2d<s32>((s32)EnemyPosition->X, (s32)EnemyPosition->Y), 
				vector2d<s32>(EnemyPosition->X + 64, EnemyPosition->Y + 64), angleToFacePlayer(enemyCollision,rocketCollision), vector2df(1, 1), 1, SColor(255, 255, 255, 255));

			drawCollision(rocketCollision, driver);
			drawCollision(enemyCollision, driver);

			draw2DImageWithRot(driver, rocket, rect<s32>(currentColumn * rocketAnimFrameSizeW, row * rocketAnimFrameSizeH,
				(currentColumn + 1) * rocketAnimFrameSizeW, (row + 1) * rocketAnimFrameSizeH), 
				position2d<s32>((s32)RocketPosition.X, (s32)RocketPosition.Y), 
				vector2d<s32>(RocketPosition.X + rocketAnimFrameSizeW / 2, RocketPosition.Y + rocketAnimFrameSizeH / 2), rotationAngle, vector2df(1, 1), 1, SColor(255, 255, 255, 255));
				
			if (fire)
			{
				driver->draw2DLine(vector2d<s32>(enemyCollision->center.x, enemyCollision->center.y), vector2d<s32>(rocketCollision->center.x, rocketCollision->center.y), SColor(255, 250, 250, 0));
			}
			driver->endScene();


		int fps = driver->getFPS();

		if (LastFps != fps)
		{
			core::stringw stingFPS = L"FPS: ";
			stingFPS += fps;
			stingFPS += L" / Current State: ";
			stingFPS += currentState;
			device->setWindowCaption(stingFPS.c_str());
			LastFps = fps;
		}
	}

	device->drop();
	soundEngine->drop();

	return 0;
}

/*
That's all. I hope it was not too difficult.
**/

