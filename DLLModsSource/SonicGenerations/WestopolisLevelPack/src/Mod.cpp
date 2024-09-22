#include "customObjects/CPointLightOneShot.h"
#include "customObjects/CRedFruit.h"
#include "customObjects/CWestopolisLaser.h"

extern "C" __declspec(dllexport) void Init() {
	using namespace Mod;

	BB_INSTALL_SET_OBJECT_MAKE_HOOK(CPointLightOneShot)
	BB_INSTALL_SET_OBJECT_MAKE_HOOK(CRedFruit)
	BB_INSTALL_SET_OBJECT_MAKE_HOOK(CWestopolisLaser)
}

extern "C" __declspec(dllexport) void OnFrame() {
	//Mod::onFrame();
}

extern "C" __declspec(dllexport) void PostInit() {
    // Force Enable Post-Processing on Particles
	uint32_t toCheck = *(uint32_t*) 0x13DD790;
	
	if (toCheck != 2) {
	    // Append Render Particle's last child to Render Scene Base
	    const auto renderSceneBase = reinterpret_cast<Hedgehog::FxRenderFramework::SDrawInstanceParam*>(0x13DDC88);

	    const auto newChildren = new Hedgehog::FxRenderFramework::SDrawInstanceParam[renderSceneBase->ChildParamCount + 1];
	    memcpy(newChildren, renderSceneBase->ChildParams, renderSceneBase->ChildParamCount * sizeof(Hedgehog::FxRenderFramework::SDrawInstanceParam));
	    memcpy(&newChildren[renderSceneBase->ChildParamCount], reinterpret_cast<void*>(0x13DC8C8), sizeof(Hedgehog::FxRenderFramework::SDrawInstanceParam));

	    // Pass new children
	    WRITE_MEMORY(&renderSceneBase->ChildParams, void*, newChildren);
	    WRITE_MEMORY(&renderSceneBase->ChildParamCount, uint32_t, renderSceneBase->ChildParamCount + 1);

	    // Render only framebuffer/stencil particles in Render Particle
	    WRITE_MEMORY(0x13DD790, uint32_t, 2);
	}
}