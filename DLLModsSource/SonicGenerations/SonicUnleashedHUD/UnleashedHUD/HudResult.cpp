#include "HudResult.h"

boost::shared_ptr<Sonic::CGameObjectCSD> spResult;
Chao::CSD::RCPtr<Chao::CSD::CProject> rcProjectResult;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcResultTitle;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcResultNum[HudResult::ResultNumType::COUNT];
Chao::CSD::RCPtr<Chao::CSD::CScene> rcResultNewRecordTime;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcResultNewRecordScore;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcResultRankText;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcResultRank;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcResultFooter;
Chao::CSD::RCPtr<Chao::CSD::CScene> rcResultFooterReplay;

bool m_isWerehog = false; // TODO: move this to configuration
float m_resultTimer = 0.0f;
HudResult::ResultState m_resultState = HudResult::ResultState::Idle;
HudResult::ResultState m_resultStateNew = HudResult::ResultState::Idle;

float m_stageTime = 0.0f;
HudResult::ResultData m_resultData;
HudResult::StageData m_stageData;
HudResult::ResultSoundState m_soundState;

HOOK(void, __fastcall, HudResult_MsgStartGoalResult, 0x10B58A0, uint32_t* This, void* Edx, void* message)
{
	// Don't run the message itself
}

void __fastcall HudResult_CHudResultRemoveCallback(Sonic::CGameObject* This, void*, Sonic::CGameDocument* pGameDocument)
{
	if (spResult)
	{
		printf("[Unleashed HUD] Result destroyed\n");
		spResult->SendMessage(spResult->m_ActorID, boost::make_shared<Sonic::Message::MsgKill>());
		spResult = nullptr;
	}

	Chao::CSD::CProject::DestroyScene(rcProjectResult.Get(), rcResultTitle);
	for (int i = 0; i < HudResult::ResultNumType::COUNT; i++)
	{
		if (rcResultNum[i])
		{
			Chao::CSD::CProject::DestroyScene(rcProjectResult.Get(), rcResultNum[i]);
		}
	}
	Chao::CSD::CProject::DestroyScene(rcProjectResult.Get(), rcResultNewRecordTime);
	Chao::CSD::CProject::DestroyScene(rcProjectResult.Get(), rcResultNewRecordScore);
	Chao::CSD::CProject::DestroyScene(rcProjectResult.Get(), rcResultRankText);
	Chao::CSD::CProject::DestroyScene(rcProjectResult.Get(), rcResultRank);
	Chao::CSD::CProject::DestroyScene(rcProjectResult.Get(), rcResultFooter);
	Chao::CSD::CProject::DestroyScene(rcProjectResult.Get(), rcResultFooterReplay);

	rcProjectResult = nullptr;
}

HOOK(int, __fastcall, HudResult_CStateGoalFadeBeforeBegin, 0xCFE080, uint32_t* This)
{
	int result = originalHudResult_CStateGoalFadeBeforeBegin(This);
	{
		m_resultTimer = 0.0f;
		m_resultState = HudResult::ResultState::Idle;
		m_resultStateNew = HudResult::ResultState::Idle;

		m_stageTime = *(float*)Common::GetMultiLevelAddress(This[2] + 0x60, { 0x8, 0x184 });
		m_resultData = *(HudResult::ResultData*)(This[2] + 0x16C);
	}
	return result;
}

HOOK(void, __fastcall, HudResult_CStateGoalFadeInEnd, 0xCFA470, hh::fnd::CStateMachineBase::CStateBase* This)
{
	originalHudResult_CStateGoalFadeInEnd(This);

	// Result starts
	if (This->m_Time > *(float*)0x1A426B8)
	{
		m_resultTimer = 0.0f;
		m_resultStateNew = HudResult::ResultState::MainWait;
	}
}

HOOK(int, __fastcall, HudResult_CHudResultAddCallback, 0x10B8ED0, Sonic::CGameObject* This, void* Edx, int a2, int a3, int a4)
{
	int result = originalHudResult_CHudResultAddCallback(This, Edx, a2, a3, a4);
	HudResult_CHudResultRemoveCallback(This, nullptr, nullptr);

	printf("[Unleashed HUD] Result created\n");
	Sonic::CCsdDatabaseWrapper wrapper(This->m_pMember->m_pGameDocument->m_pMember->m_spDatabase.get());

	auto spCsdProject = wrapper.GetCsdProject("ui_result_swa");
	rcProjectResult = spCsdProject->m_rcProject;

	rcResultTitle = rcProjectResult->CreateScene("result_title");
	rcResultTitle->SetHideFlag(true);

	for (int i = 0; i < 6; i++)
	{
		rcResultNum[i] = rcProjectResult->CreateScene((std::string("result_num_") + std::to_string(i + 1)).c_str());
		rcResultNum[i]->SetHideFlag(true);
	}

	if (rcProjectResult && !spResult)
	{
		spResult = boost::make_shared<Sonic::CGameObjectCSD>(rcProjectResult, 0.5f, "HUD", false);
		Sonic::CGameDocument::GetInstance()->AddGameObject(spResult, "main", This);
	}

	return result;
}

void HudResult_PlayMotion(Chao::CSD::RCPtr<Chao::CSD::CScene>& scene, char const* motion, bool loop = false)
{
	scene->SetHideFlag(false);
	scene->SetMotion(motion);
	scene->SetMotionFrame(0.0f);
	scene->m_MotionDisableFlag = false;
	scene->m_MotionSpeed = 1.0f;
	scene->m_MotionRepeatType = loop ? Chao::CSD::eMotionRepeatType_Loop : Chao::CSD::eMotionRepeatType_PlayOnce;
	scene->Update();
}

float const cResultMainDelay = 0.2833f;
float const cResultNewRecordDelay = 1.6667f;
float const cResultRankDelay = 0.9f;
float const cResultFooterDelay = 2.0f;
bool hasNewRecord = false;
HOOK(void, __fastcall, HudResult_CHudResultAdvance, 0x10B96D0, Sonic::CGameObject* This, void* Edx, const hh::fnd::SUpdateInfo& in_rUpdateInfo)
{
	originalHudResult_CHudResultAdvance(This, Edx, in_rUpdateInfo);
	if (*(uint32_t*)0x10B96E6 != 0xFFD285E8)
	{
		// We are finished
		HudResult_CHudResultRemoveCallback(This, nullptr, nullptr);
		WRITE_MEMORY(0x10B96E6, uint8_t, 0xE8, 0x85, 0xD2, 0xFF, 0xFF);
		return;
	}

	FUNCTION_PTR(bool, __cdecl, IsFirstTimePlayStage, 0x10B7BB0);

	// New states
	if (m_resultState != m_resultStateNew)
	{
		char const* motion_so_ev = m_isWerehog ? "Intro_ev_Anim" : "Intro_so_Anim";

		m_resultState = m_resultStateNew;
		switch (m_resultState)
		{
		case HudResult::ResultState::Idle:
		{
			printf("[Unleashed HUD] Result State: Idle\n");
			break;
		}
		case HudResult::ResultState::MainWait:
		{
			printf("[Unleashed HUD] Result State: Main Wait\n");

			m_soundState = HudResult::ResultSoundState();
			break;
		}
		case HudResult::ResultState::Main:
		{
			printf("[Unleashed HUD] Result State: Main\n");

			HudResult_PlayMotion(rcResultTitle, motion_so_ev);

			for (int i = 0; i < HudResult::ResultNumType::COUNT; i++)
			{
				switch (i)
				{
				case HudResult::ResultNumType::TIME:
				{
					int millisecond = (int)(m_stageTime * 100.0f) % 100;
					int second = (int)m_stageTime % 60;
					int minute = (int)m_stageTime / 60;
					static char buffer[16];
					sprintf(buffer, "%02d:%02d:%02d", minute, second, millisecond);
					rcResultNum[i]->GetNode("time_num")->SetText(buffer);
					break;
				}
				case HudResult::ResultNumType::RINGS:
				{
					// TODO: Change this to match Generations if ScoreGen is not enabled
					auto const* context = Sonic::Player::CPlayerSpeedContext::GetInstance();
					int score = context->m_RingCount * 100;
					rcResultNum[i]->GetNode("num_2")->SetText(std::to_string(score).c_str());
					break;
				}
				case HudResult::ResultNumType::SPEED:
				{
					// TODO:
					break;
				}
				case HudResult::ResultNumType::ENEMY:
				{
					// TODO:
					break;
				}
				case HudResult::ResultNumType::TRICKS:
				{
					// TODO:
					break;
				}
				case HudResult::ResultNumType::TOTAL:
				{
					rcResultNum[i]->GetNode("num_6")->SetText(std::to_string(m_resultData.m_score).c_str());
					break;
				}
				default: break;
				}

				HudResult_PlayMotion(rcResultNum[i], motion_so_ev);
			}
			break;
		}
		case HudResult::ResultState::NewRecord:
		{
			printf("[Unleashed HUD] Result State: New Record\n");

			hasNewRecord = false;
			constexpr float newRecordX = 0.628125f * 1280.0f;

			// New record data, based on sub_10B7F00
			rcResultNewRecordTime = rcProjectResult->CreateScene("result_newR");
			if (((1 << (0 & 0x1F)) & *(uint32_t*)((uint32_t)This + 580 + 4 * (0 >> 5))) != 0)
			{
				rcResultNewRecordTime->SetPosition(newRecordX, 0.32083333f * 720.0f);
				HudResult_PlayMotion(rcResultNewRecordTime, "motion_so_ev");
				hasNewRecord = true;
			}
			else
			{
				rcResultNewRecordTime->SetHideFlag(true);
			}

			rcResultNewRecordScore = rcProjectResult->CreateScene("result_newR");
			if (((1 << (2 & 0x1F)) & *(uint32_t*)((uint32_t)This + 580 + 4 * (2 >> 5))) != 0)
			{
				rcResultNewRecordScore->SetPosition(newRecordX, 0.7861111f * 720.0f);
				HudResult_PlayMotion(rcResultNewRecordScore, "motion_so_ev");
				hasNewRecord = true;
			}
			else
			{
				rcResultNewRecordScore->SetHideFlag(true);
			}

			break;
		}
		case HudResult::ResultState::Rank:
		{
			printf("[Unleashed HUD] Result State: Rank\n");

			rcResultRankText = rcProjectResult->CreateScene("result_rank");
			HudResult_PlayMotion(rcResultRankText, "Intro_Anim");

			uint32_t cueID;
			std::string rankSceneName;
			switch (m_resultData.m_perfectRank)
			{
			case HudResult::ResultRankType::S: rankSceneName = "result_rank_S"; break;
			case HudResult::ResultRankType::A: rankSceneName = "result_rank_A"; break; 
			case HudResult::ResultRankType::B: rankSceneName = "result_rank_B"; break; 
			case HudResult::ResultRankType::C: rankSceneName = "result_rank_C"; break; 
			case HudResult::ResultRankType::D: rankSceneName = "result_rank_D"; break; 
			default: rankSceneName = "result_rank_E"; break;
			}

			rcResultRank = rcProjectResult->CreateScene(rankSceneName.c_str());
			HudResult_PlayMotion(rcResultRank, "Intro_Anim");
			break;
		}
		case HudResult::ResultState::Footer:
		{
			printf("[Unleashed HUD] Result State: Footer\n");

			rcResultFooter = rcProjectResult->CreateScene("result_footer");

			if (!IsFirstTimePlayStage())
			{
				rcResultFooterReplay = rcProjectResult->CreateScene("result_footer");
				rcResultFooterReplay->SetPosition(-200.0f, 0.0f);
				rcResultFooterReplay->GetNode("btn_A")->SetPatternIndex(3);
				rcResultFooterReplay->GetNode("next_txt")->SetPatternIndex(1);
			}
			break;
		}
		}

		m_resultTimer = 0.0f;
	}

	// Loop animation for rank S,A,B
	if (rcResultRank && rcResultRank->m_MotionDisableFlag &&
	(m_resultData.m_perfectRank == HudResult::ResultRankType::S || m_resultData.m_perfectRank == HudResult::ResultRankType::A || m_resultData.m_perfectRank == HudResult::ResultRankType::B))
	{
		HudResult_PlayMotion(rcResultRank, "Usual_Anim", true);
	}

	m_resultTimer += in_rUpdateInfo.DeltaTime;

	// Bar sliding sfx
	if (m_resultState == HudResult::ResultState::Main)
	{
		static float barTime[] = { 1.0f, 1.18333f, 1.3333f, 1.45f };
		for (int i = 0; i < 4; i++)
		{
			if (m_resultTimer > barTime[i] && !m_soundState.m_bar[i])
			{
				m_soundState.m_bar[i] = true;
				static SharedPtrTypeless barSoundHandle;
				Common::PlaySoundStatic(barSoundHandle, 1000020);
			}
		}
	}

	// Total/New record sfx
	if (m_resultState == HudResult::ResultState::NewRecord)
	{
		if (rcResultNewRecordScore && rcResultNewRecordScore->m_MotionFrame >= 3.0f && !m_soundState.m_total)
		{
			m_soundState.m_total = true;

			if (hasNewRecord)
			{
				static SharedPtrTypeless newRecordSoundHandle;
				Common::PlaySoundStatic(newRecordSoundHandle, 1000053);
			}

			static SharedPtrTypeless totalSoundHandle;
			Common::PlaySoundStatic(totalSoundHandle, 1000021);
		}
	}

	// Rank slam/rank quote sfx
	if (m_resultState == HudResult::ResultState::Rank)
	{
		if (rcResultRank && rcResultRank->m_MotionFrame >= 9.0f && !m_soundState.m_rank)
		{
			m_soundState.m_rank = true;

			uint32_t cueID;
			switch (m_resultData.m_perfectRank)
			{
			case HudResult::ResultRankType::S: cueID = 1000041; break;
			case HudResult::ResultRankType::A: cueID = 1000042; break;
			case HudResult::ResultRankType::B: cueID = 1000043; break;
			case HudResult::ResultRankType::C: cueID = 1000044; break;
			case HudResult::ResultRankType::D: cueID = 1000045; break;
			default: cueID = 1000046; break;
			}

			static SharedPtrTypeless rankSoundHandle;
			Common::PlaySoundStatic(rankSoundHandle, cueID);
		}

		if (!Common::IsPlayerSuper() && rcResultRank && rcResultRank->m_MotionFrame >= 40.0f && !m_soundState.m_rankVoice)
		{
			m_soundState.m_rankVoice = true;

			uint32_t cueID;
			switch (m_resultData.m_perfectRank)
			{
			case HudResult::ResultRankType::S: cueID = 40000; break;
			case HudResult::ResultRankType::A: cueID = 40001; break;
			case HudResult::ResultRankType::B: cueID = 40002; break;
			case HudResult::ResultRankType::C: cueID = 40003; break;
			case HudResult::ResultRankType::D: cueID = 40004; break;
			default: cueID = 40005; break;
			}

			static SharedPtrTypeless rankVoiceHandle;
			Common::PlaySoundStatic(rankVoiceHandle, cueID);
		}
	}
	

	// State transition
	switch (m_resultState)
	{
	case HudResult::ResultState::Idle: break;
	case HudResult::ResultState::MainWait:
	{
		if (m_resultTimer > cResultMainDelay)
		{
			m_resultStateNew = HudResult::ResultState::Main;
		}
		break;
	}
	case HudResult::ResultState::Main:
	{
		if (m_resultTimer > cResultNewRecordDelay)
		{
			m_resultStateNew = HudResult::ResultState::NewRecord;
		}
		break;
	}
	case HudResult::ResultState::NewRecord:
	{
		if (m_resultTimer > cResultRankDelay)
		{
			m_resultStateNew = HudResult::ResultState::Rank;
		}
		break;
	}
	case HudResult::ResultState::Rank:
	{
		if (m_resultTimer > cResultFooterDelay)
		{
			m_resultStateNew = HudResult::ResultState::Footer;
		}
		break;
	}
	case HudResult::ResultState::Footer:
	{
		Sonic::SPadState const* padState = &Sonic::CInputState::GetInstance()->GetPadState();
		if (padState->IsTapped(Sonic::EKeyState::eKeyState_A))
		{
			WRITE_JUMP(0x10B96E6, (void*)0x10B974B);
		}
		else if (!IsFirstTimePlayStage() && padState->IsTapped(Sonic::EKeyState::eKeyState_Y))
		{
			WRITE_JUMP(0x10B96E6, (void*)0x10B999F);
		}

		break;
	}
	case HudResult::ResultState::FadeOut: break;
	}
}

void HudResult::Install()
{
	// Use Unleashed goal camera (default) param
	// CameraSp -> CameraSu (this doesn't read CameraSu, just fall back to default values)
	WRITE_MEMORY(0x15AF4C3, uint8_t, 0x75);
	WRITE_MEMORY(0x15D1EBB, uint8_t, 0x75);
	WRITE_MEMORY(0x15D293B, uint8_t, 0x75);
	WRITE_MEMORY(0x1A48C7C, float, -0.6053f); // -0.08 from default OffsetRight

	// Prevent using Gen's result
	INSTALL_HOOK(HudResult_MsgStartGoalResult);

	// Data collection
	INSTALL_HOOK(HudResult_CStateGoalFadeBeforeBegin);
	INSTALL_HOOK(HudResult_CStateGoalFadeInEnd);

	// Result HUD handling
	INSTALL_HOOK(HudResult_CHudResultAddCallback);
	WRITE_MEMORY(0x16A1C38, void*, HudResult_CHudResultRemoveCallback);
	INSTALL_HOOK(HudResult_CHudResultAdvance);

}