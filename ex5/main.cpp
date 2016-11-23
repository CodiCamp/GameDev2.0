#include <irrlicht.h>
#include "MyEventReceiver.h"
using namespace std;

using namespace irr;
using namespace core;
using namespace video;

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif


/*
At first, we let the user select the driver type, then start up the engine, set
a caption, and get a pointer to the video driver.
*/

int main()
{
	s32 rocketAnimFrameSizeW = 55;
	s32 rocketAnimFrameSizeH = 83;

	//create an instance of the event receiver
	MyEventReceiver receiver;

	// create device
	IrrlichtDevice *device = createDevice(video::EDT_DIRECT3D9,
		core::dimension2d<u32>(800, 600), 16, false, false, false, &receiver);

	if (device == 0)
		return 1; // could not create selected driver.

	video::IVideoDriver* driver = device->getVideoDriver();
	//add scene manager
	scene::ISceneManager* smgr = device->getSceneManager();
	
	video::ITexture* rocket = driver->getTexture("./media/Rocket_spritesheet.png");
	//driver->makeColorKeyTexture(image, position2d<s32>(0, 0));

	video::ITexture* bgrnd = driver->getTexture("./media/Background_Purple_Space-800x600.jpg");
	//driver->makeColorKeyTexture(bgrnd, position2d<s32>(0, 0));
	
	video::ITexture* sun = driver->getTexture("./media/sunAnim.PNG");
	driver->makeColorKeyTexture(sun, position2d<s32>(0, 0));
	


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
	
	position2df RocketPosition(300,300);

	while (device->run())
	{
		const u32 currentTime = device->getTimer()->getTime();
		//u32 Dt = currentTime - lastTime;

		const f32 Dt= (f32)(currentTime - lastTime); // Time in seconds
		lastTime = currentTime;

		//compute how much time has passed since previous frame. If it is > the time between animation frames, curentframe++
		/* Check if keys W, S, A or D are being held down, and move the
		sphere node around respectively. */


		if (receiver.IsKeyDown(irr::KEY_KEY_W))
		{
			
			//RocketPosition.X -= Dt* f32(cos(rotationAngle));
			//RocketPosition.Y += Dt* f32(sin(rotationAngle));
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

//		node->setPosition(nodePosition);
		
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

			/*driver->draw2DImage(rocket,
			position2d<s32>((s32)RocketPosition.X, (s32)RocketPosition.Y),
				rect<s32>(currentColumn * rocketAnimFrameSizeW, row * rocketAnimFrameSizeH,
				(currentColumn + 1) * rocketAnimFrameSizeW, (row + 1) * rocketAnimFrameSizeH),0,
				SColor(255, 255, 255, 255), true);*/
			driver->draw2DImage(driver, rocket, rect<s32>(currentColumn * rocketAnimFrameSizeW, row * rocketAnimFrameSizeH,
				(currentColumn + 1) * rocketAnimFrameSizeW, (row + 1) * rocketAnimFrameSizeH), position2d<s32>((s32)RocketPosition.X, (s32)RocketPosition.Y), vector2d<s32>(RocketPosition.X + rocketAnimFrameSizeW / 2, RocketPosition.Y + rocketAnimFrameSizeH / 2), rotationAngle, vector2df(1, 1), 1, SColor(255, 255, 255, 255));
		
		//smgr->addCameraSceneNode();
		driver->endScene();

		int fps = driver->getFPS();

		if (LastFps != fps)
		{
			core::stringw stingFPS = L"FPS: ";
			stingFPS += fps;
			device->setWindowCaption(stingFPS.c_str());
			LastFps = fps;
		}
	}

	device->drop();

	return 0;
}

/*
That's all. I hope it was not too difficult.
**/

