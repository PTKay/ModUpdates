#pragma once
#include "../include/Types.h"
#include "../include/SoundPoint.h"
#include "../include/Glitter.h"

namespace Mod
{
    class CWestopolisLaser : public Sonic::SoundPoint::CObjSoundPoint
    {
    public:
        BB_SET_OBJECT_MAKE("WestopolisLaser");

    private:
        // Need a local copy of GlitterPlayer now since we're no longer inheriting CObjectBase
        Sonic::CGlitterPlayer* m_pGlitterPlayer = nullptr;
        float m_damageTime = 0.0f;
        float m_maxDamageTime = 0.5f;
        
        bool m_IsInsideDamageCollision = false;

    public:
        struct LightParam
        {
            float FlashTime = 0.3f;
            float StartTime = 0.5f;
            int EaseType = 0;

            float RangeNear = 2.0f;
            float RangeFar = 10.0f;

            float Brightness = 5.0f;
            CVector4 Color = {1.0f, 0.8f, 0.6f, 1.0f};

            bool IsActive = false;

            void Flash(float deltaTime);
            Hedgehog::Base::CRefPtr<Sonic::CLocalLight> m_Light;

            LightParam(float flashTime, float startTime, int easeType, float rangeNear, float rangeFar,
                       float brightness, CVector4 color)
                : FlashTime(flashTime), StartTime(startTime), EaseType(easeType), RangeNear(rangeNear),
                  Brightness(brightness), Color(color)
            {
            }

        private:
            float m_InternalTime = 0.0f;
            float m_PlaybackTime = 0.0f;
        };

        LightParam Light1Delay
        {
            0.6f, 0.0f, 0, 7.0f, 10.0f, 1.0f, CVector4(1.0f, 0.87f, 0.0f, 1.0f)
        };
        LightParam Light1Laser
        {
            1.25f, 0.9f, 2, 7.0f, 10.0f, 1.5f, CVector4(0.39f, 0.64f, 1.0f, 1.0f)
        };
        LightParam Light2Laser
        {
            1.25f, 0, 2, 7.0f, 10.0f, 1.5f, CVector4(0.39f, 0.64f, 1.0f, 1.0f)
        };

        bool IsInstant = false;
        

        void InitializeEditParam(Sonic::CEditParam& in_rEditParam) override;
        void AddCallback(const Hedgehog::Base::THolder<Sonic::CWorld>& in_rWorldHolder,
                         Sonic::CGameDocument* in_pGameDocument,
                         const boost::shared_ptr<Hedgehog::Database::CDatabase>& in_spDatabase) override;
        void UpdateParallel(const Hedgehog::Universe::SUpdateInfo& in_rUpdateInfo) override;
        bool ProcessMessage(Hedgehog::Universe::Message& message, bool flag);
        void KillCallback() override;
    };

    BB_SET_OBJECT_MAKE_HOOK(CWestopolisLaser)
}
