#pragma once

class HudResult
{
public:
	enum ResultState : int
	{
		Idle,
		MainWait,
		Main,
		NewRecord,
		Rank,
		Footer,
		FadeOut,
	};

	enum ResultNumType : int
	{
		TIME,
		RINGS,
		SPEED,
		ENEMY,
		TRICKS,
		TOTAL,
		COUNT
	};

	enum ResultRankType : int
	{
		E = -1,
		D,
		C,
		B,
		A,
		S
	};

	struct ResultData
	{
		int m_score;
		ResultRankType m_rank;
		ResultRankType m_perfectRank;
		int m_nextRankTime;
		float m_totalProp;	// result progress bar (time prop + ring prop) 
		float m_timeProp;	// result progress bar (time prop)
	};

	struct ResultSoundState
	{
		bool m_bar[ResultNumType::COUNT];
		bool m_total;
		bool m_rank;
		bool m_rankVoice;

		ResultSoundState()
			: m_bar{false, false, false, false, false, false}
			, m_total(false)
			, m_rank(false)
			, m_rankVoice(false)
		{}
	};

	struct StageData
	{
		float m_ringScore;
		float m_speedScore;
		float m_enemyScore;
		float m_trickScore;

		StageData()
			: m_ringScore(0.0f)
			, m_speedScore(0.0f)
			, m_enemyScore(0.0f)
			, m_trickScore(0.0f)
		{}
	};

	static void Install();
};

