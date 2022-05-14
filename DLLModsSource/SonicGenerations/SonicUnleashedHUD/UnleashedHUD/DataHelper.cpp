#include <memory>
#include <windows.h>

#include "detours\include\detours.h"
#include "include\Helpers.h"
#include "Controllers\Controller.h"
#include "include\Common.h"
#include "include\ScoreGenerationsAPI.h"

/**
	The game's process. Used in ForceWriteData()
**/
const HANDLE process = GetCurrentProcess();

/**
	The chosen button prompts. Defaults to XBOX360
**/
int buttonPrompts = XBOX360;

/**
	If it's to fix the extended boost gauge. Defaults to false
**/
bool fixExtended = false;

/**
	Changes permissions of a memory zone, writes on it,
	and then restores the previous permissions.
**/
bool ForceWriteData(void* address, const char* data, size_t size) {
	DWORD old;
	bool result = false;

	result = VirtualProtect(address, size, PAGE_READWRITE, &old);
	WriteProcessMemory(process, address, data, size, nullptr);
	VirtualProtect(address, size, old, &old);
	return result;
}

void WriteButtons(ControllerInfo info) {
	ForceWriteData((void*)0x015E3D80, info.btn, 15);
	ForceWriteData((void*)0x016A6AA4, info.btn, 15);

	ForceWriteData((void*)0x0168C928, info.win, 9);
	ForceWriteData((void*)0x016A6B9C, info.win, 9);

	ForceWriteData((void*)0x016A93CC, info.pam, 6);
	ForceWriteData((void*)0x0168B294, info.pam, 6);
	ForceWriteData((void*)0x0168B3C8, info.pam, 6);
	ForceWriteData((void*)0x0168B780, info.pam, 6);

	ForceWriteData((void*)0x01579524, info.trick, 11);
	ForceWriteData((void*)0x016D84F8, info.trick, 11);

	ForceWriteData((void*)0x01688344, info.how, 8);
	ForceWriteData((void*)0x016886A8, info.how, 8);
	ForceWriteData((void*)0x01692BC4, info.how, 8);

	ForceWriteData((void*)0x0154BCA4, info.bt, 12);
	ForceWriteData((void*)0x0154CF74, info.bt, 12);
	ForceWriteData((void*)0x0168BD44, info.bt, 12);
}

void WriteData(int buttonType) {
	buttonPrompts = buttonType;
	ControllerInfo info = GetXncpNames(buttonType, false, false);

	WriteButtons(info);

	// WriteUIGameplay
	ForceWriteData((void*)0x0168E328, info.ui, 11);

	ForceWriteData((void*)0x0168F1EC, "ui_gp_signul", 12);			// Used to add Unleashed's Ready GO animation without breaking missions.
	ForceWriteData((void*)0x0155E5D8, "ui_lockon_cursar", 16);		// Used to keep the original Generations lock on cursor in the Time Eater boss battle.
}

int GetButtonPrompts() {
	return buttonPrompts;
}

Chao::CSD::RCPtr<Chao::CSD::CProject> rcPlayScreen;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcPlayerCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcTimeCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcRingCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcSpeedGauge;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcRingEnergyGauge;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcGaugeFrame;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcScoreCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcSpeedCount;
boost::shared_ptr<Sonic::CGameObjectCSD> spPlayScreen;

Chao::CSD::RCPtr<Chao::CSD::CProject> rcMissionScreen;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcPosition;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcCountdown;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcItemCount;
boost::shared_ptr<Sonic::CGameObjectCSD> spMissionScreen;

Chao::CSD::RCPtr<Chao::CSD::CProject> rcPlayScreenEv;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcRingGet;
boost::shared_ptr<Sonic::CGameObjectCSD> spPlayScreenEv;

size_t prevRingCount;
bool isMission;
size_t itemCountDenominator;
float speed;

boost::shared_ptr<Hedgehog::Sound::CSoundHandle> spSpeed01;
boost::shared_ptr<Hedgehog::Sound::CSoundHandle> spSpeed02[3];
boost::shared_ptr<Hedgehog::Sound::CSoundHandle> spSpeed03;

void CreateScreen(Sonic::CGameObject* pParentGameObject)
{
	if (rcPlayScreen && !spPlayScreen)
		pParentGameObject->m_pMember->m_pGameDocument->AddGameObject(spPlayScreen = boost::make_shared<Sonic::CGameObjectCSD>(rcPlayScreen, 0.5f, "HUD_B1", false), "main", pParentGameObject);

	if (rcMissionScreen && !spMissionScreen)
		pParentGameObject->m_pMember->m_pGameDocument->AddGameObject(spMissionScreen = boost::make_shared<Sonic::CGameObjectCSD>(rcMissionScreen, 0.5f, "HUD_B1", false), "main", pParentGameObject);

	if (rcPlayScreenEv && !spPlayScreenEv)
		pParentGameObject->m_pMember->m_pGameDocument->AddGameObject(spPlayScreenEv = boost::make_shared<Sonic::CGameObjectCSD>(rcPlayScreenEv, 0.5f, "HUD_B1", false), "main", pParentGameObject);
}

void KillScreen()
{
	if (spPlayScreen)
	{
		spPlayScreen->SendMessage(spPlayScreen->m_ActorID, boost::make_shared<Sonic::Message::MsgKill>());
		spPlayScreen = nullptr;
	}

	if (spMissionScreen)
	{
		spMissionScreen->SendMessage(spMissionScreen->m_ActorID, boost::make_shared<Sonic::Message::MsgKill>());
		spMissionScreen = nullptr;
	}

	if (spPlayScreenEv)
	{
		spPlayScreenEv->SendMessage(spPlayScreenEv->m_ActorID, boost::make_shared<Sonic::Message::MsgKill>());
		spPlayScreenEv = nullptr;
	}
}

void ToggleScreen(const bool visible, Sonic::CGameObject* pParentGameObject)
{
	if (visible)
		CreateScreen(pParentGameObject);
	else
		KillScreen();
}

hh::math::CVector2 SetMissionScenePosition(Chao::CSD::CScene* pScene, const size_t index)
{
	char name[4];
	sprintf(name, "%02d", index);
	const auto position = rcPosition->GetNode(name)->GetPosition();
	pScene->SetPosition(position.x(), position.y());
	return position;
}

void FreezeMotion(Chao::CSD::CScene* pScene, bool end = true)
{
	pScene->SetMotionFrame(end ? pScene->m_MotionEndFrame : 0.0f);
	pScene->m_MotionSpeed = 0.0f;
	pScene->m_MotionRepeatType = Chao::CSD::eMotionRepeatType_PlayOnce;
	pScene->m_MotionDisableFlag = true;
}

void GetTime(Sonic::CGameDocument* pGameDocument, size_t* minutes, size_t* seconds, size_t* milliseconds)
{
	static uint32_t pAddr = 0xD61570;
	__asm
	{
		mov ecx, minutes
		mov edi, seconds
		mov esi, milliseconds
		mov eax, pGameDocument
		call[pAddr]
	}
}

float GetTime(Sonic::CGameDocument* pGameDocument)
{
	const auto pMember = (uint8_t*)pGameDocument->m_pMember;
	return max(0, max(0, *(float*)(pMember + 0x184)) + *(float*)(pMember + 0x18C));
}

void __fastcall CHudSonicStageRemoveCallback(Sonic::CGameObject* This, void*, Sonic::CGameDocument* pGameDocument)
{
	KillScreen();

	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcPlayerCount);

	if (isMission)
		Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcTimeCount);
	else
		Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcTimeCount);

	if (isMission)
		Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcRingCount);
	else
		Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcRingCount);

	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcSpeedGauge);
	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcRingEnergyGauge);
	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcGaugeFrame);
	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcScoreCount);

	if (rcSpeedCount)
		Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcSpeedCount);

	Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcPosition);
	Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcCountdown);
	Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcItemCount);

	Chao::CSD::CProject::DestroyScene(rcPlayScreenEv.Get(), rcRingGet);

	rcPlayScreen = nullptr;
	rcMissionScreen = nullptr;
	rcPlayScreenEv = nullptr;
	isMission = false;
}

HOOK(void, __fastcall, ProcMsgGetMissionLimitTime, 0xD0F0E0, Sonic::CGameObject* This, void* Edx, hh::fnd::Message& in_rMsg)
{
	originalProcMsgGetMissionLimitTime(This, Edx, in_rMsg);
	if (!rcCountdown)
		return;

	const float limitTime = *(float*)((char*)&in_rMsg + 16);
	const float elapsedTime = GetTime(**This->m_pMember->m_pGameDocument);
	const float remainingTime = limitTime - elapsedTime;

	char text[16];
	sprintf(text, "%02d", (int)(remainingTime * 100.0f) % 100);
	rcCountdown->GetNode("time001_l")->SetText(text);

	sprintf(text, "%02d", (int)remainingTime % 60);
	rcCountdown->GetNode("time010_l")->SetText(text);

	sprintf(text, "%02d", (int)(remainingTime / 60));
	rcCountdown->GetNode("time100_l")->SetText(text);

	rcCountdown->SetMotionFrame(elapsedTime / limitTime * rcCountdown->m_MotionEndFrame);
}

HOOK(void, __fastcall, ProcMsgGetMissionCondition, 0xD0F130, Sonic::CGameObject* This, void* Edx, hh::fnd::Message& in_rMsg)
{
	originalProcMsgGetMissionCondition(This, Edx, in_rMsg);
	itemCountDenominator = *(size_t*)((char*)&in_rMsg + 20);
}

HOOK(void, __fastcall, ProcMsgNotifyLapTimeHud, 0x1097640, Sonic::CGameObject* This, void* Edx, hh::fnd::Message& in_rMsg)
{
	if (!rcPlayScreen)
		return;

	rcSpeedCount = rcPlayScreen->CreateScene("speed_count");
	rcSpeedCount->m_MotionRepeatType = Chao::CSD::eMotionRepeatType_PlayOnce;

	const auto playerContext = Sonic::Player::CPlayerSpeedContext::GetInstance();
	speed = playerContext->GetVelocity().norm() / (playerContext->m_Is2DMode ? 45.0f : 90.0f) * 1000.0f;

	spSpeed01 = nullptr;
	for (size_t i = 0; i < _countof(spSpeed02); i++)
		spSpeed02[i] = nullptr;

	spSpeed03 = nullptr;
}

HOOK(void, __fastcall, CCountdownUpdate, 0x124F360, void* This, void* Edx, const hh::fnd::SUpdateInfo& in_rUpdateInfo)
{
	const auto& rcScene = *(Chao::CSD::RCPtr<Chao::CSD::CScene>*)(*(char**)((char*)This + 0xAC) + 0x14);
	rcScene->SetHideFlag(true);
	originalCCountdownUpdate(This, Edx, in_rUpdateInfo);
}

HOOK(void, __fastcall, CHudSonicStageDelayProcessImp, 0x109A8D0, Sonic::CGameObject* This)
{
	ScoreGenerationsAPI::SetVisibility(false);
	originalCHudSonicStageDelayProcessImp(This);
	CHudSonicStageRemoveCallback(This, nullptr, nullptr);

	Sonic::CCsdDatabaseWrapper wrapper(This->m_pMember->m_pGameDocument->m_pMember->m_spDatabase.get());

	auto spCsdProject = wrapper.GetCsdProject("ui_playscreen");
	rcPlayScreen = spCsdProject->m_rcProject;

	spCsdProject = wrapper.GetCsdProject("ui_missionscreen");
	rcMissionScreen = spCsdProject->m_rcProject;

	spCsdProject = wrapper.GetCsdProject("ui_playscreen_ev");
	rcPlayScreenEv = spCsdProject->m_rcProject;

	rcPosition = rcMissionScreen->CreateScene("position");

	isMission = !Common::IsCurrentStageBossBattle() && (Common::GetCurrentStageID() & (SMT_Mission1 | SMT_Mission2 | SMT_Mission3 | SMT_Mission4 | SMT_Mission5));

	size_t& flags = ((size_t*)This)[151];

	if (flags & 0x1) // Lives
		rcPlayerCount = rcPlayScreen->CreateScene("player_count");

	if (flags & 0x2000) // Countdown
	{
		rcCountdown = rcMissionScreen->CreateScene("time_count", "conditional_timer_so");
		FreezeMotion(rcCountdown.Get());
	}

	if (flags & 0x2 && !rcCountdown) // Time
	{
		if (isMission)
			rcTimeCount = rcMissionScreen->CreateScene("time_count", "normal_so");
		else
			rcTimeCount = rcPlayScreen->CreateScene("time_count");
	}

	if (flags & 0x20000) // Mission Target
	{
		rcItemCount = rcMissionScreen->CreateScene("item_count", "conditional_meet_so");
		FreezeMotion(rcItemCount.Get());

		rcItemCount->GetNode("icons")->SetHideFlag(true);

		char text[16];
		sprintf(text, "%03d", itemCountDenominator);
		rcItemCount->GetNode("num_deno")->SetText(text);
	}

	if (flags & 0x200) // Boost Gauge
	{
		rcSpeedGauge = rcPlayScreen->CreateScene("so_speed_gauge");
		rcRingEnergyGauge = rcPlayScreen->CreateScene("so_ringenagy_gauge");
		rcGaugeFrame = rcPlayScreen->CreateScene("gauge_frame");

		FreezeMotion(rcSpeedGauge.Get());
		FreezeMotion(rcRingEnergyGauge.Get());
		FreezeMotion(rcGaugeFrame.Get());
	}

	if (ScoreGenerationsAPI::IsAttached() && !ScoreGenerationsAPI::IsStageForbidden()) // Score
		rcScoreCount = rcPlayScreen->CreateScene("score_count");

	if (flags & 0x4 || Common::GetCurrentStageID() == SMT_bsd) // Rings
	{
		// Re-purpose score for Classic
		if (!rcSpeedGauge)
		{
			if (isMission)
				rcRingCount = rcMissionScreen->CreateScene("score_count", rcCountdown ? "conditional_meet_so" : "normal_so");
			else 
				rcRingCount = rcPlayScreen->CreateScene("score_count");

			rcRingCount->GetNode("txt")->SetPatternIndex(1);

			const auto& rcTxtLarge = rcRingCount->GetNode("txt_l");
			if (rcTxtLarge)
				rcTxtLarge->SetPatternIndex(1);

			rcRingGet = rcPlayScreenEv->CreateScene("ring_get");
			if (rcCountdown)
				rcRingGet->GetNode("ring_img")->SetScale(0.72f, 0.72f);

			if (rcScoreCount)
			{
				rcRingCount->SetPosition(0, 50);
				rcRingGet->SetPosition(0, 50);
			}

			FreezeMotion(rcRingCount.Get(), false);
			FreezeMotion(rcRingGet.Get());
		}
		else
			rcRingCount = rcPlayScreen->CreateScene("ring_count");
	}

	flags &= ~(0x1 | 0x2 | 0x4 | 0x200 | 0x800 | 0x1000000); // Mask to prevent crash when game tries accessing the elements we disabled later on

	CreateScreen(This);
}

HOOK(void, __fastcall, CHudSonicStageUpdateParallel, 0x1098A50, Sonic::CGameObject* This, void* Edx, const hh::fnd::SUpdateInfo& in_rUpdateInfo)
{
	originalCHudSonicStageUpdateParallel(This, Edx, in_rUpdateInfo);

	ToggleScreen(*(bool*)0x1A430D8, This); // ms_IsRenderGameMainHud

	if (!spPlayScreen)
		return;

	char text[256];
	size_t rowIndex = 1;

	if (rcPlayerCount)
	{
		const size_t liveCountAddr = Common::GetMultiLevelAddress(0x1E66B34, { 0x4, 0x1B4, 0x7C, 0x9FDC });
		if (liveCountAddr)
		{
			sprintf(text, "%02d", *(size_t*)liveCountAddr);
			rcPlayerCount->GetNode("player")->SetText(text);
		}
	}

	if (rcTimeCount)
	{
		size_t minutes, seconds, milliseconds;
		GetTime(**This->m_pMember->m_pGameDocument, &minutes, &seconds, &milliseconds);

		sprintf(text, "%02d", milliseconds);
		rcTimeCount->GetNode("time001")->SetText(text);

		sprintf(text, "%02d", seconds);
		rcTimeCount->GetNode("time010")->SetText(text);

		sprintf(text, "%02d", minutes);
		rcTimeCount->GetNode("time100")->SetText(text);

		if (isMission)
			SetMissionScenePosition(rcTimeCount.Get(), rowIndex++);
	}

	if (rcCountdown)
		SetMissionScenePosition(rcCountdown.Get(), rowIndex++);

	const auto playerContext = Sonic::Player::CPlayerSpeedContext::GetInstance();

	if (rcSpeedGauge && playerContext)
		rcSpeedGauge->SetMotionFrame(playerContext->m_HorizontalVelocity.norm() / (playerContext->m_Is2DMode ? 45.0f : 90.0f) * 100.0f);

	if (rcRingEnergyGauge && playerContext)
	{
		rcRingEnergyGauge->SetMotion("total_quantity");
		rcRingEnergyGauge->SetMotionFrame(100.0f);
		rcRingEnergyGauge->Update(0.0f);

		playerContext->m_ChaosEnergy = min(playerContext->m_ChaosEnergy, playerContext->GetMaxChaosEnergy());

		rcRingEnergyGauge->SetMotion("size");
		rcRingEnergyGauge->SetMotionFrame(playerContext->m_ChaosEnergy / playerContext->GetMaxChaosEnergy() * 100.0f);
		rcRingEnergyGauge->Update(0.0f);
	}

	if (rcGaugeFrame)
	{
		rcGaugeFrame->SetMotionFrame(100.0f);
	}

	if (rcScoreCount)
	{
		sprintf(text, "%08d", ScoreGenerationsAPI::GetScore());
		rcScoreCount->GetNode("score")->SetText(text);
	}

	if (rcRingCount && playerContext)
	{
		sprintf(text, "%03d", playerContext->m_RingCount);

		// Classic
		if (!rcSpeedGauge)
		{
			rcRingCount->GetNode("score")->SetText(text);

			const auto& rcScoreLarge = rcRingCount->GetNode("score_l");
			if (rcScoreLarge)
				rcScoreLarge->SetText(text);

			if (isMission)
			{
				const auto position = SetMissionScenePosition(rcRingCount.Get(), rowIndex++);
				rcRingGet->SetPosition(position.x() - (rcCountdown ? 106 : 128), position.y() - (rcCountdown ? 199.5f : 200));
			}

			if (prevRingCount < playerContext->m_RingCount && rcRingGet->m_MotionDisableFlag)
			{
				rcRingGet->SetMotion("get_Anim");
				rcRingGet->SetMotionFrame(0.0f);
				rcRingGet->m_MotionDisableFlag = false;
				rcRingGet->m_MotionRepeatType = Chao::CSD::eMotionRepeatType_PlayOnce;
				rcRingGet->m_MotionSpeed = 1.0f;
				rcRingGet->Update();
			}
		}

		// Modern
		else
		{
			rcRingCount->GetNode("num_ring")->SetText(text);

			if (prevRingCount < playerContext->m_RingCount)
				spPlayScreen->m_rcProject->CreateScene("ring_get")->m_MotionRepeatType = Chao::CSD::eMotionRepeatType_PlayThenDestroy;
		}

		prevRingCount = playerContext->m_RingCount;
	}

	if (rcItemCount)
	{
		const size_t count = *(size_t*)((char*)This + 0x300);
		sprintf(text, "%03d", count);
		rcItemCount->GetNode("num_nume")->SetText(text);
		rcItemCount->SetMotionFrame(count >= itemCountDenominator ? rcItemCount->m_MotionEndFrame : 0.0f);

		SetMissionScenePosition(rcItemCount.Get(), rowIndex++);

		for (size_t i = 0; i < 2; i++)
		{
			const auto& rcMissionTarget = ((Chao::CSD::RCPtr<Chao::CSD::CScene>*)((char*)This + 0x130))[i];
			if (!rcMissionTarget)
				continue;
			
			rcMissionTarget->GetNode("bg")->SetHideFlag(true);
			rcMissionTarget->GetNode("SXY")->SetHideFlag(true);
			rcMissionTarget->GetNode("num_deno")->SetHideFlag(true);
			rcMissionTarget->GetNode("img_slash")->SetHideFlag(true);

			const auto& rcImgIcon = rcMissionTarget->GetNode("img_icon");
			rcImgIcon->SetPosition(0, 0);
			rcMissionTarget->Update();
			const auto position = rcItemCount->GetNode("ring")->GetPosition() - rcImgIcon->GetPosition();
			rcImgIcon->SetPosition(position.x(), position.y());
		}
	}

	if (rcSpeedCount)
	{
		if (rcSpeedCount->m_MotionDisableFlag)
			Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcSpeedCount);

		else
		{
			const float frames[] = { 87, 105, 123, 145 };

			if (rcSpeedCount->m_MotionFrame < frames[0])
			{
				if (!spSpeed01)
					spSpeed01 = playerContext->PlaySound(39, 0);
			}
			else
				spSpeed01 = nullptr;

			sprintf(text, " %03d", rand() % 1000);
			sprintf(text + 16, "%04d", (int)speed);

			for (size_t i = 0; i < 4; i++)
			{
				if (rcSpeedCount->m_MotionFrame > frames[i])
				{
					text[3 - i] = text[19 - i];

					auto& rHandle = i < 3 ? spSpeed02[i] : spSpeed03;
					if (!rHandle)
						rHandle = playerContext->PlaySound(i < 3 ? 40 : 41, 0);
				}
			}

			char* pText = text[0] == ' ' ? text + 1 : text;

			rcSpeedCount->GetNode("num_speed")->SetText(pText);
			rcSpeedCount->GetNode("num_speed_deep")->SetText(pText);
			rcSpeedCount->GetNode("num_speed_pale")->SetText(pText);
		}
	}
}

void HookFunctions()
{
	INSTALL_HOOK(ProcMsgGetMissionLimitTime);
	INSTALL_HOOK(ProcMsgGetMissionCondition);
	INSTALL_HOOK(ProcMsgNotifyLapTimeHud);
	INSTALL_HOOK(CHudSonicStageDelayProcessImp);
	INSTALL_HOOK(CCountdownUpdate);
	INSTALL_HOOK(CHudSonicStageUpdateParallel);
	WRITE_MEMORY(0x16A467C, void*, CHudSonicStageRemoveCallback);

	WRITE_MEMORY(0x109B1A4, uint8_t, 0x90, 0xE9); // Disable lives
	WRITE_MEMORY(0x109B490, uint8_t, 0x90, 0xE9); // Disable time
	WRITE_MEMORY(0x109B5AD, uint8_t, 0x90, 0xE9); // Disable rings
	WRITE_MEMORY(0x109B8F5, uint8_t, 0x90, 0xE9); // Disable boost gauge
	WRITE_MEMORY(0x109BC88, uint8_t, 0x90, 0xE9); // Disable boost button
	//WRITE_MEMORY(0x109BEF0, uint8_t, 0x90, 0xE9); // Disable mission countdown
	WRITE_MEMORY(0x109C3E2, uint8_t, 0x90, 0xE9); // Disable mission rank
}

extern "C" __declspec(dllexport) void PostInit()
{
	ScoreGenerationsAPI::SetVisibility(false);
}