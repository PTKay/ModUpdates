Chao::CSD::RCPtr<Chao::CSD::CProject> rcPlayScreen;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcPlayerCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcTimeCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcRingCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcSpeedGauge;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcRingEnergyGauge;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcGaugeFrame;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcScoreCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcSpeedCount;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcInfoCustom;
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
size_t actionCount;
hh::math::CVector2 infoCustomPos;
bool scoreEnabled;

float xAspectOffset = 0.0f;
float yAspectOffset = 0.0f;

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

void HandleInfoCustom(const Chao::CSD::RCPtr<Chao::CSD::CScene>& rcScene)
{
	constexpr float scale = 0.9f;

	if (const auto rcIconCustom = rcScene->GetNode("icon_custom_0"))
		rcIconCustom->SetScale(scale, scale);

	if (const auto rcIconCustom = rcScene->GetNode("icon_chao_5"))
		rcIconCustom->SetScale(scale, scale);

	rcScene->GetNode("position")->SetScale(0.8f, 0.8f);
	rcScene->GetNode("bg")->SetHideFlag(true);
	rcScene->GetNode("icon_btn")->SetHideFlag(true);
	rcScene->GetNode("brilliance")->SetHideFlag(true);
	rcScene->SetPosition(infoCustomPos.x() + (rcCountdown ? 34.0f : -15.0f), infoCustomPos.y() - 103.0f);
}

const Chao::CSD::RCPtr<Chao::CSD::CScene>& GetGpSonicSafeScene(void* This)
{
	return *(Chao::CSD::RCPtr<Chao::CSD::CScene>*)(*(char**)((char*)This + 0xAC) + 0x14);
}

void CreateRingGet()
{
	if (rcRingGet)
		return;

	rcRingGet = rcPlayScreenEv->CreateScene("ring_get");
	if (rcCountdown)
		rcRingGet->GetNode("ring_img")->SetScale(0.72f, 0.72f);

	FreezeMotion(rcRingGet.Get());
}

void SetRingGetPosition(const hh::math::CVector2& position, float offset = 0.0f)
{
	if (rcRingGet)
		rcRingGet->SetPosition(position.x() + offset - (rcCountdown ? 106 : 128), position.y() - (rcCountdown ? 199.5f : 200));
}

void __declspec(naked) GetScoreEnabled()
{
	static uint32_t returnAddress = 0x109C254;
	__asm
	{
		mov	scoreEnabled, 1
		jmp[returnAddress]
	}
}

HOOK(void, __fastcall, ProcMsgSetPinballHud, 0x1095D40, Sonic::CGameObject* This, void* Edx, hh::fnd::Message& in_rMsg)
{
	// Update score
	if ((*((char*)&in_rMsg + 16) & 1) && rcScoreCount)
	{
		char text[32];
		sprintf(text, "%08d", *(size_t*)((char*)&in_rMsg + 20));
		rcScoreCount->GetNode("score")->SetText(text);
		rcScoreCount->Update();
	}

	originalProcMsgSetPinballHud(This, Edx, in_rMsg);
}

void __fastcall CHudSonicStageRemoveCallback(Sonic::CGameObject* This, void*, Sonic::CGameDocument* pGameDocument)
{
	KillScreen();

	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcPlayerCount);

	if (isMission)
		Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcTimeCount);
	else
		Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcTimeCount);

	if (isMission && !rcSpeedGauge)
		Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcRingCount);
	else
		Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcRingCount);

	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcSpeedGauge);
	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcRingEnergyGauge);
	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcGaugeFrame);
	Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcScoreCount);

	if (rcSpeedCount)
		Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcSpeedCount);

	if (isMission)
		Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcInfoCustom);
	else
		Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcInfoCustom);

	Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcPosition);
	Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcCountdown);
	Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcItemCount);

	Chao::CSD::CProject::DestroyScene(rcPlayScreenEv.Get(), rcRingGet);

	rcPlayScreen = nullptr;
	rcMissionScreen = nullptr;
	rcPlayScreenEv = nullptr;
	isMission = false;
	actionCount = 1;
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
	rcSpeedCount->SetPosition(xAspectOffset, 0);
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
	GetGpSonicSafeScene(This)->SetHideFlag(true);
	originalCCountdownUpdate(This, Edx, in_rUpdateInfo);
}

HOOK(void, __fastcall, CLastBossGaugeNewUpdate, 0x124E930, void* This, void* Edx, const hh::fnd::SUpdateInfo& in_rUpdateInfo)
{
	HandleInfoCustom(GetGpSonicSafeScene(This));
	originalCLastBossGaugeNewUpdate(This, Edx, in_rUpdateInfo);
}

HOOK(void, __fastcall, ProcMsgChangeCustomHud, 0x1096FF0, Sonic::CGameObject* This, void* Edx, hh::fnd::Message& in_rMsg)
{
	size_t* spriteIndex = (size_t*)((char*)&in_rMsg + 16);
	size_t* spriteCount = (size_t*)((char*)&in_rMsg + 20);

	if (rcInfoCustom)
	{
		actionCount = *spriteCount > 0 ? max(actionCount, *spriteCount) : 0;

		char text[16];

		sprintf(text, "%d", *spriteCount);
		rcInfoCustom->GetNode("num_nume")->SetText(text);

		sprintf(text, "%d", actionCount);
		rcInfoCustom->GetNode("num_deno")->SetText(text);
	}

	*spriteCount = *spriteCount > 0 ? 1 : 0;

	originalProcMsgChangeCustomHud(This, Edx, in_rMsg);
}

HOOK(void, __fastcall, CHudSonicStageDelayProcessImp, 0x109A8D0, Sonic::CGameObject* This)
{
	scoreEnabled = false;
	originalCHudSonicStageDelayProcessImp(This);
	CHudSonicStageRemoveCallback(This, nullptr, nullptr);

	if (*(size_t*)0x6F23C6 != 0x75D8C0D9) // Widescreen Support
	{
		const float aspect = (float)*(size_t*)0x1DFDDDC / (float)*(size_t*)0x1DFDDE0;

		if (aspect * 9.0f > 16.0f)
		{
			xAspectOffset = 720.0f * aspect - 1280.0f;
			yAspectOffset = 0.0f;
		}
		else
		{
			xAspectOffset = 0.0f;
			yAspectOffset = 1280.0f / aspect - 720.0f;
		}
	}
	else 
	{
		xAspectOffset = 0.0f;
		yAspectOffset = 0.0f;
	}

	Sonic::CCsdDatabaseWrapper wrapper(This->m_pMember->m_pGameDocument->m_pMember->m_spDatabase.get());

	auto spCsdProject = wrapper.GetCsdProject("ui_playscreen");
	rcPlayScreen = spCsdProject->m_rcProject;

	spCsdProject = wrapper.GetCsdProject("ui_missionscreen");
	rcMissionScreen = spCsdProject->m_rcProject;

	spCsdProject = wrapper.GetCsdProject("ui_playscreen_ev");
	rcPlayScreenEv = spCsdProject->m_rcProject;

	rcPosition = rcMissionScreen->CreateScene("position");
	rcPosition->SetPosition(0, 0);

	isMission = !Common::IsCurrentStageBossBattle() && (Common::GetCurrentStageID() & (SMT_Mission1 | SMT_Mission2 | SMT_Mission3 | SMT_Mission4 | SMT_Mission5));

	size_t& flags = ((size_t*)This)[151];

	float offset = 0.0f;

	if (flags & 0x1 && *(uint8_t*)0x1098C82 != 0xEB) // Lives (accounts for Disable Lives patch)
	{
		rcPlayerCount = rcPlayScreen->CreateScene("player_count");
		rcPlayerCount->SetPosition(0.0f, 0.0f);
	}
	else
		offset = -50.0f;

	if (flags & 0x2000) // Countdown
	{
		rcCountdown = rcMissionScreen->CreateScene("time_count", "conditional_timer_so");
		FreezeMotion(rcCountdown.Get());
	}

	if (!rcCountdown)
	{
		if (flags & 0x2) // Time
		{
			if (isMission)
				rcTimeCount = rcMissionScreen->CreateScene("time_count", "normal_so");
			else
			{
				rcTimeCount = rcPlayScreen->CreateScene("time_count");
				rcTimeCount->SetPosition(0, offset);
			}
		}
	}
	else
	{
		offset += -50.0f;
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

		rcSpeedGauge->SetPosition(0, yAspectOffset);
		rcRingEnergyGauge->SetPosition(0, yAspectOffset);
		rcGaugeFrame->SetPosition(0, yAspectOffset);

		FreezeMotion(rcSpeedGauge.Get());
		FreezeMotion(rcRingEnergyGauge.Get());
		FreezeMotion(rcGaugeFrame.Get());
	}

	if (scoreEnabled) // Score
	{
		rcScoreCount = rcPlayScreen->CreateScene("score_count");
		rcScoreCount->SetPosition(0, offset);
	}

	if (flags & 0x400204 || Common::GetCurrentStageID() == SMT_bsd) // Rings
	{
		// Re-purpose score for Classic
		if (!rcSpeedGauge)
		{
			if (isMission)
				rcRingCount = rcMissionScreen->CreateScene("score_count", rcCountdown ? "conditional_meet_so" : "normal_so");
			else
				rcRingCount = rcPlayScreen->CreateScene("score_count");

			rcRingCount->GetNode("txt")->SetPatternIndex(1);

			if (const auto rcTxtLarge = rcRingCount->GetNode("txt_l"))
				rcTxtLarge->SetPatternIndex(1);

			CreateRingGet();

			rcRingCount->SetPosition(0, offset + (rcScoreCount ? 50 : 0));
			rcRingGet->SetPosition(0, offset + (rcScoreCount ? 50 : 0));

			FreezeMotion(rcRingCount.Get(), false);
		}
		else
		{
			rcRingCount = rcPlayScreen->CreateScene("ring_count");
			rcRingCount->SetPosition(0, yAspectOffset);
		}
	}

	if (isMission)
	{
		rcInfoCustom = rcMissionScreen->CreateScene(rcCountdown ? "item_count" : "item_count_s", "conditional_meet_so");
		rcInfoCustom->SetHideFlag(true);
		rcInfoCustom->GetNode("txt")->SetPatternIndex(1);

		if (const auto rcIcons = rcInfoCustom->GetNode("icons"))
			rcIcons->SetHideFlag(true);
	}
	else
	{
		rcInfoCustom = rcPlayScreen->CreateScene("item_count", "conditional_meet_so");

		float infoCustomOffset = offset + (rcScoreCount ? 50 : 0);
		infoCustomOffset += !rcSpeedGauge && rcRingCount ? 50 : 0;

		infoCustomPos.x() = 128.0f;
		infoCustomPos.y() = 200 + infoCustomOffset;

		rcInfoCustom->SetPosition(0, infoCustomOffset);
	}

	FreezeMotion(rcInfoCustom.Get(), false);

	flags &= ~(0x1 | 0x2 | 0x400004 | 0x200 | 0x800 | 0x1000000); // Mask to prevent crash when game tries accessing the elements we disabled later on

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

	const size_t flags = ((size_t*)This)[151];

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
		rcGaugeFrame->SetMotionFrame(100.0f);

	if (rcRingCount && playerContext)
	{
		sprintf(text, "%03d", playerContext->m_RingCount);

		// Classic
		if (!rcSpeedGauge)
		{
			rcRingCount->GetNode("score")->SetText(text);

			if (const auto rcScoreLarge = rcRingCount->GetNode("score_l"))
				rcScoreLarge->SetText(text);

			if (isMission)
				SetRingGetPosition(SetMissionScenePosition(rcRingCount.Get(), rowIndex++));
		}

		// Modern
		else
			rcRingCount->GetNode("num_ring")->SetText(text);
	}

	if (rcRingGet && playerContext && prevRingCount < playerContext->m_RingCount && rcRingGet->m_MotionDisableFlag)
	{
		rcRingGet->SetMotion("get_Anim");
		rcRingGet->SetMotionFrame(0.0f);
		rcRingGet->m_MotionDisableFlag = false;
		rcRingGet->m_MotionRepeatType = Chao::CSD::eMotionRepeatType_PlayOnce;
		rcRingGet->m_MotionSpeed = 1.0f;
		rcRingGet->Update();
	}

	if (playerContext)
		prevRingCount = playerContext->m_RingCount;
	else
		prevRingCount = 0;

	if (rcInfoCustom)
	{
		const bool multiple = *(size_t*)((char*)This + 572) > 0;
		const bool single = *(bool*)((char*)This + 580) || strcmp((const char*)0x1E774D4, "blb") == 0;
		const bool visible = multiple || single;

		rcInfoCustom->SetHideFlag(!visible);

		if (visible)
		{
			if (single && !(flags & 0x100)) // Ignore Cream mission
			{
				rcInfoCustom->GetNode("num_nume")->SetText("1");
				rcInfoCustom->GetNode("num_deno")->SetText("1");
			}

			if (isMission)
				infoCustomPos = SetMissionScenePosition(rcInfoCustom.Get(), rowIndex++);

			for (size_t i = 0; i < 2; i++)
			{
				const auto& rcScene = ((Chao::CSD::RCPtr<Chao::CSD::CScene>*)((char*)This + 0x1C8))[i];
				if (rcScene)
					HandleInfoCustom(rcScene);
			}
		}
	}

	if (rcItemCount)
	{
		const size_t count = *(size_t*)((char*)This + 0x300);
		sprintf(text, "%03d", count);
		rcItemCount->GetNode("num_nume")->SetText(text);
		rcItemCount->SetMotionFrame(count >= itemCountDenominator ? rcItemCount->m_MotionEndFrame : 0.0f);

		const auto position = SetMissionScenePosition(rcItemCount.Get(), rowIndex++);

		// Mission ring count (shamelessly taken from Brian)
		const bool missionRingCount = *(bool*)((uint32_t)This + 620);
		
		if (missionRingCount)
		{
			const size_t prevMissionRingCount = *(size_t*)((char*)This + 768);
			if (prevMissionRingCount != playerContext->m_RingCount)
			{
				*(size_t*)((char*)This + 772) = prevMissionRingCount;
				*(size_t*)((char*)This + 768) = playerContext->m_RingCount;
				*(size_t*)((char*)This + 764) = 1;
			}

			// We don't need the actual ring count if it's shown here anyway
			if (rcRingCount && !rcSpeedGauge)
			{
				if (isMission)
					Chao::CSD::CProject::DestroyScene(rcMissionScreen.Get(), rcRingCount);
				else
					Chao::CSD::CProject::DestroyScene(rcPlayScreen.Get(), rcRingCount);
			}

			CreateRingGet();
			SetRingGetPosition(position, 32);
		}

		for (size_t i = 0; i < 2; i++)
		{
			const auto& rcMissionTarget = ((Chao::CSD::RCPtr<Chao::CSD::CScene>*)((char*)This + 0x130))[i];
			if (!rcMissionTarget)
				continue;

			if (missionRingCount)
			{
				rcMissionTarget->SetHideFlag(true);
				continue;
			}

			rcMissionTarget->GetNode("bg")->SetHideFlag(true);
			rcMissionTarget->GetNode("SXY")->SetHideFlag(true);
			rcMissionTarget->GetNode("num_deno")->SetHideFlag(true);
			rcMissionTarget->GetNode("img_slash")->SetHideFlag(true);

			const auto rcImgIcon = rcMissionTarget->GetNode("img_icon");
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
					spSpeed01 = playerContext->PlaySound(1000016, 0);
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
						rHandle = playerContext->PlaySound(i < 3 ? 1000017 : 1000018, 0);
				}
			}

			char* pText = text[0] == ' ' ? text + 1 : text;

			rcSpeedCount->GetNode("num_speed")->SetText(pText);
			rcSpeedCount->GetNode("num_speed_deep")->SetText(pText);
			rcSpeedCount->GetNode("num_speed_pale")->SetText(pText);
		}
	}

	// Hide pin_score
	if (const auto rcPinScore = *(Chao::CSD::RCPtr<Chao::CSD::CScene>*)((char*)This + 0x128))
		rcPinScore->SetHideFlag(true);
}

class CObjGetItem : public Sonic::CGameObject
{
	hh::math::CVector m_Position;
	hh::math::CVector4 m_ScreenPosition;
	hh::math::CQuaternion m_Rotation;
	float m_Factor{};
	boost::shared_ptr<hh::mr::CSingleElement> m_spModel;

public:
	CObjGetItem(const hh::math::CVector& in_rPosition, const hh::math::CQuaternion& in_rRotation)
		: m_Position(in_rPosition), m_Rotation(in_rRotation) {}

	void AddCallback(const Hedgehog::Base::THolder<Sonic::CWorld>& worldHolder, Sonic::CGameDocument* pGameDocument,
		const boost::shared_ptr<Hedgehog::Database::CDatabase>& spDatabase) override
	{
		Sonic::CApplicationDocument::GetInstance()->AddMessageActor("GameObject", this);
		pGameDocument->AddUpdateUnit("f", this);

		m_spModel = boost::make_shared<hh::mr::CSingleElement>(hh::mr::CMirageDatabaseWrapper(spDatabase.get()).GetModelData("cmn_obj_ring_HD"));
		AddRenderable("HUD_B2", m_spModel, false);

		const auto spCamera = pGameDocument->GetWorld()->GetCamera();
		const auto viewPosition = spCamera->m_MyCamera.m_View * Eigen::Vector3f(m_Position);

		m_ScreenPosition = spCamera->m_MyCamera.m_Projection * hh::math::CVector4(viewPosition.x(), viewPosition.y(), viewPosition.z(), 1.0f);
		m_ScreenPosition /= m_ScreenPosition.w();

		m_Rotation = spCamera->m_MyCamera.m_View.rotation() * m_Rotation;
	}

	void UpdateParallel(const Hedgehog::Universe::SUpdateInfo& updateInfo) override
	{
		const auto spCamera = m_pMember->m_pGameDocument->GetWorld()->GetCamera();

		const hh::math::CVector4 ringDepth = spCamera->m_MyCamera.m_Projection * hh::math::CVector4(0, 0, -10.0f, 1.0f);

		const hh::math::CVector4 viewPosInProj = spCamera->m_MyCamera.m_Projection.inverse() *
			((1.0f - m_Factor) * m_ScreenPosition + m_Factor * hh::math::CVector4(-0.7765625f, -0.7833333333333333f, ringDepth.z() / ringDepth.w(), 1.0f));

		const hh::math::CVector viewPosition = viewPosInProj.head<3>() / viewPosInProj.w();
		
		m_spModel->m_spInstanceInfo->m_Transform = spCamera->m_MyCamera.m_View.inverse() * (Eigen::Translation3f(viewPosition) * m_Rotation);
		m_Rotation = m_Rotation.slerp(updateInfo.DeltaTime * 6.0f, hh::math::CQuaternion::Identity());

		m_Factor += updateInfo.DeltaTime * 0.125f;
		m_Factor *= 1.2732395f;

		if (m_Factor > 1.0f)
		{
			SendMessage(m_ActorID, boost::make_shared<Sonic::Message::MsgKill>());

			const auto rcScene = spPlayScreen->m_rcProject->CreateScene("ring_get");
			rcScene->SetPosition(0, yAspectOffset);
			rcScene->m_MotionRepeatType = Chao::CSD::eMotionRepeatType_PlayThenDestroy;
		}
	}
};

HOOK(void, __fastcall, CObjRingProcMsgHitEventCollision, 0x10534B0, Sonic::CGameObject3D* This, void* Edx, hh::fnd::Message& in_rMsg)
{
	if (rcSpeedGauge)
	{
		This->m_pMember->m_pGameDocument->AddGameObject(boost::make_shared<CObjGetItem>(
			This->m_spMatrixNodeTransform->m_Transform.m_Position, This->m_spMatrixNodeTransform->m_Transform.m_Rotation));
	}

	originalCObjRingProcMsgHitEventCollision(This, Edx, in_rMsg);
}

void HudSonicStage::Install()
{
	INSTALL_HOOK(ProcMsgGetMissionLimitTime);
	INSTALL_HOOK(ProcMsgGetMissionCondition);
	INSTALL_HOOK(ProcMsgNotifyLapTimeHud);
	INSTALL_HOOK(CHudSonicStageDelayProcessImp);
	INSTALL_HOOK(CCountdownUpdate);
	INSTALL_HOOK(CLastBossGaugeNewUpdate);
	INSTALL_HOOK(ProcMsgChangeCustomHud);
	INSTALL_HOOK(CHudSonicStageUpdateParallel);
	INSTALL_HOOK(ProcMsgSetPinballHud);
	INSTALL_HOOK(CObjRingProcMsgHitEventCollision);

	WRITE_JUMP(0x109C1DC, GetScoreEnabled);

	WRITE_STATIC_MEMORY(0x155E5D8, "ui_lockon_cursar", 16); // Used to keep the original Generations lock on cursor in the Time Eater boss battle.
	WRITE_STATIC_MEMORY(0x168F1EC, "ui_gp_signul", 12);	    // Used to add Unleashed's Ready GO animation without breaking missions.

	WRITE_MEMORY(0xDEBCA4, uint8_t, 0xEB);
	WRITE_MEMORY(0x109B1A4, uint8_t, 0xE9, 0xDC, 0x02, 0x00, 0x00); // Disable lives (patched differently to not clash with Disable Lives patch)
	WRITE_MEMORY(0x109B490, uint8_t, 0x90, 0xE9); // Disable time
	WRITE_MEMORY(0x109B5AD, uint8_t, 0x90, 0xE9); // Disable rings
	WRITE_MEMORY(0x109B6A7, uint8_t, 0x90, 0xE9); // Disable final boss rings
	WRITE_MEMORY(0x109B8F5, uint8_t, 0x90, 0xE9); // Disable boost gauge
	WRITE_MEMORY(0x109BC88, uint8_t, 0x90, 0xE9); // Disable boost button
	// WRITE_MEMORY(0x109BEF0, uint8_t, 0x90, 0xE9); // Disable mission countdown
	WRITE_MEMORY(0x109C3E2, uint8_t, 0x90, 0xE9); // Disable mission rank
	WRITE_MEMORY(0x16A467C, void*, CHudSonicStageRemoveCallback);
}