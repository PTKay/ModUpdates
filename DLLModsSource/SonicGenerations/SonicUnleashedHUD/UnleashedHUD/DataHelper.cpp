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

Chao::CSD::RCPtr<Chao::CSD::CProject> rcProject;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcPlayerCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcTimeCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcRingCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcSpeedGauge;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcRingEnergyGauge;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcGaugeFrame;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcScoreCount;
boost::shared_ptr<Sonic::CGameObjectCSD> spPlayScreen;
size_t prevRingCount;

void CreatePlayScreen(Sonic::CGameObject* pParentGameObject)
{
	if (!rcProject || spPlayScreen)
		return;

	pParentGameObject->m_pMember->m_pGameDocument->AddGameObject(spPlayScreen = boost::make_shared<Sonic::CGameObjectCSD>(rcProject, 0.5f, "HUD", false), "main", pParentGameObject);
}

void KillPlayScreen()
{
	if (!spPlayScreen)
		return;

	void* obj = spPlayScreen.get();
	static uint32_t pAddr = 0xD5FD10;
	__asm
	{
		mov edi, obj
		call[pAddr]
	}

	spPlayScreen = nullptr;
}

void TogglePlayScreen(const bool visible, Sonic::CGameObject* pParentGameObject)
{
	if (visible == !!spPlayScreen)
		return;

	if (visible)
		CreatePlayScreen(pParentGameObject);
	else
		KillPlayScreen();
}

void TogglePlayScreen(Sonic::CGameObject* pParentGameObject)
{
	TogglePlayScreen(!spPlayScreen, pParentGameObject);
}

void __fastcall CHudSonicStageRemoveCallback(Sonic::CGameObject* This, void*, Sonic::CGameDocument* pGameDocument)
{
	if (rcProject)
	{
		rcProject->KillScene(rcPlayerCount);
		rcProject->KillScene(rcTimeCount);
		rcProject->KillScene(rcRingCount);
		rcProject->KillScene(rcSpeedGauge);
		rcProject->KillScene(rcRingEnergyGauge);
		rcProject->KillScene(rcGaugeFrame);
		rcProject->KillScene(rcScoreCount);
		rcProject = nullptr;
	}
	else
	{
		rcProject = nullptr;
		rcPlayerCount = nullptr;
		rcTimeCount = nullptr;
		rcRingCount = nullptr;
		rcSpeedGauge = nullptr;
		rcRingEnergyGauge = nullptr;
		rcGaugeFrame = nullptr;
		rcScoreCount = nullptr;
	}

	KillPlayScreen();
}

HOOK(void, __fastcall, CHudSonicStageDelayProcessImp, 0x109A8D0, Sonic::CGameObject* This)
{
	ScoreGenerationsAPI::SetVisibility(false);
	originalCHudSonicStageDelayProcessImp(This);
	CHudSonicStageRemoveCallback(This, nullptr, nullptr);

	boost::shared_ptr<Sonic::CCsdProject> spCsdProject;
	Sonic::CCsdDatabaseWrapper(This->m_pMember->m_pGameDocument->m_pMember->m_spDatabase.get()).GetCsdProject(spCsdProject, "ui_playscreen");
	rcProject = spCsdProject->m_rcProject;

	size_t& flags = ((size_t*)This)[151];

	if (flags & 0x1) // Lives
		rcPlayerCount = spCsdProject->m_rcProject->GetScene("player_count", 0);

	if (flags & 0x2) // Time
		rcTimeCount = spCsdProject->m_rcProject->GetScene("time_count", 0);

	if (flags & 0x4 || Common::GetCurrentStageID() == SMT_bsd) // Rings
		rcRingCount = spCsdProject->m_rcProject->GetScene("ring_count", 0);

	if (flags & 0x200) // Boost Gauge
	{
		rcSpeedGauge = spCsdProject->m_rcProject->GetScene("so_speed_gauge", 0);
		rcSpeedGauge->m_MotionSpeed = 0.0f;
		rcSpeedGauge->m_MotionPlaybackType = Chao::CSD::eMotionPlaybackType_PlayOnce;

		rcRingEnergyGauge = spCsdProject->m_rcProject->GetScene("so_ringenagy_gauge", 0);
		rcRingEnergyGauge->m_MotionSpeed = 0.0f;
		rcRingEnergyGauge->m_MotionPlaybackType = Chao::CSD::eMotionPlaybackType_PlayOnce;

		rcGaugeFrame = spCsdProject->m_rcProject->GetScene("gauge_frame", 0);
		rcGaugeFrame->m_MotionSpeed = 0.0f;
		rcGaugeFrame->m_MotionPlaybackType = Chao::CSD::eMotionPlaybackType_PlayOnce;
	}

	flags &= ~(0x1 | 0x2 | 0x4 | 0x200 | 0x800); // Mask to prevent crash when game tries accessing the elements we disabled later on

	if (ScoreGenerationsAPI::IsAttached() && !ScoreGenerationsAPI::IsStageForbidden()) // Score
		rcScoreCount = spCsdProject->m_rcProject->GetScene("score_count", 0);

	CreatePlayScreen(This);
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

HOOK(void, __fastcall, CHudSonicStageUpdateParallel, 0x1098A50, Sonic::CGameObject* This, void* Edx, const hh::fnd::SUpdateInfo& in_rUpdateInfo)
{
	TogglePlayScreen(*(bool*)0x1A430D8, This); // ms_IsRenderGameMainHud

	if (!spPlayScreen)
		return originalCHudSonicStageUpdateParallel(This, Edx, in_rUpdateInfo);

	char text[256];

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
	}

	const auto playerContext = Sonic::Player::CPlayerSpeedContext::GetInstance();

	if (rcRingCount && playerContext)
	{
		sprintf(text, "%03d", playerContext->m_RingCount);
		rcRingCount->GetNode("num_ring")->SetText(text);

		if (prevRingCount < playerContext->m_RingCount)
			spPlayScreen->m_rcProject->GetScene("ring_get", 0)->m_MotionPlaybackType = Chao::CSD::eMotionPlaybackType_PlayThenHide;

		prevRingCount = playerContext->m_RingCount;
	}

	if (rcSpeedGauge && playerContext)
		rcSpeedGauge->SetMotionTime(playerContext->m_HorizontalVelocity.norm() / 90.0f * 100.0f);

	if (rcRingEnergyGauge && playerContext)
	{
		rcRingEnergyGauge->SetMotionContext("total_quantity");
		rcRingEnergyGauge->SetMotionTime(100.0f);
		rcRingEnergyGauge->Update(0.0f);

		rcRingEnergyGauge->SetMotionContext("size");
		rcRingEnergyGauge->SetMotionTime(playerContext->m_ChaosEnergy);
		rcRingEnergyGauge->Update(0.0f);
	}

	if (rcGaugeFrame)
	{
		rcGaugeFrame->SetMotionTime(100.0f);
	}

	if (rcScoreCount)
	{
		sprintf(text, "%08d", ScoreGenerationsAPI::GetScore());
		rcScoreCount->GetNode("score")->SetText(text);
	}

	originalCHudSonicStageUpdateParallel(This, Edx, in_rUpdateInfo);
}

void HookFunctions()
{
	INSTALL_HOOK(CHudSonicStageDelayProcessImp);
	INSTALL_HOOK(CHudSonicStageUpdateParallel);
	WRITE_MEMORY(0x16A467C, void*, CHudSonicStageRemoveCallback);

	WRITE_MEMORY(0x109B1A4, uint8_t, 0x90, 0xE9); // Disable lives
	WRITE_MEMORY(0x109B490, uint8_t, 0x90, 0xE9); // Disable time
	WRITE_MEMORY(0x109B5AD, uint8_t, 0x90, 0xE9); // Disable rings
	WRITE_MEMORY(0x109B8F5, uint8_t, 0x90, 0xE9); // Disable boost gauge
	WRITE_MEMORY(0x109BC88, uint8_t, 0x90, 0xE9); // Disable boost button
}

extern "C" __declspec(dllexport) void PostInit()
{
	ScoreGenerationsAPI::SetVisibility(false);
}