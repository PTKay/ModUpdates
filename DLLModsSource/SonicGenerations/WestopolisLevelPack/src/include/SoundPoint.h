#pragma once

namespace Sonic
{
    class CObjSound;
    class CObjSound3D;

#define MAKE_BASIC_OPTIMIZED_CTOR(className, address, reg) \
	static void __declspec(naked) f##className##Ctor() \
	{ \
		static constexpr int pFunc = address; \
		__asm { mov reg, ecx } \
		__asm { jmp[pFunc] } \
	} \
	static BB_FUNCTION_PTR(void, __thiscall, fp##className##Ctor, f##className##Ctor, className* This);

    MAKE_BASIC_OPTIMIZED_CTOR(CObjSound, 0x01005AA0, esi)
    MAKE_BASIC_OPTIMIZED_CTOR(CObjSound3D, 0x01005BB0, eax)

    class CObjSound : public CGameObject3D, public CSetObjectListener
    {
    public:
        CObjSound(const bb_null_ctor& nil) : CGameObject3D(nil), CSetObjectListener(nil)
        {
        }

        CObjSound() : CObjSound(bb_null_ctor{})
        {
            fpCObjSoundCtor(this);
        }

        BB_OVERRIDE_FUNCTION_PTR(void, CGameObject, UpdateParallel, 0x1004F60,
                                 (const Hedgehog::Universe::SUpdateInfo&, in_rUpdateInfo))
        BB_OVERRIDE_FUNCTION_PTR(void, CGameObject3D, AddCallback, 0x010053B0,
                                 (const Hedgehog::Base::THolder<CWorld>&, in_rWorldHolder),
                                 (Sonic::CGameDocument*, in_pGameDocument),
                                 (const boost::shared_ptr<Hedgehog::Database::CDatabase>&, in_spDatabase))
        BB_OVERRIDE_FUNCTION_PTR(bool, CMessageActor, ProcessMessage, 0x1005C00,
                                 (Hedgehog::Universe::Message&, in_rMsg), (bool, in_Flag))
        BB_OVERRIDE_FUNCTION_PTR(void, CSetObjectListener, InitializeEditParam, 0x1005730, (CEditParam&, in_rEditParam))

        //BB_OVERRIDE_FUNCTION_PTR(void, CSetObjectListener, CSetObjectListener08, 0x0, (void*,A1), (void*,A2))
        BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener0C, 0x1005000)
        BB_OVERRIDE_FUNCTION_PTR(bool, CSetObjectListener, CSetObjectListener10, 0xEB6670, (void*, A1))
        //BB_OVERRIDE_FUNCTION_PTR(void, CSetObjectListener, CSetObjectListener14, 0x0, (void*, A1))
        BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener18, 0x114E1F0)
        BB_OVERRIDE_FUNCTION_PTR(void, CSetObjectListener, CSetObjectListener1C, 0x10056C0, (void*, A1))
        BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener20, 0x1004EC0)
        BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener24, 0x01004EB0)
        BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener28, 0x1004E90)
        BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener2C, 0x1004E80)
        BB_OVERRIDE_FUNCTION_PTR(void, CSetObjectListener, CSetObjectListener30, 0x1004E70, (void*, A1))
        //BB_OVERRIDE_FUNCTION_PTR(void, CSetObjectListener, CSetObjectListener34, 0x0, (void*, A1))
        //BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener38, 0x0)
        //BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener3C, 0x0)
        //BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener40, 0x0)
        //BB_OVERRIDE_FUNCTION_PTR_NOARG(void, CSetObjectListener, CSetObjectListener44, 0x0)

        int m_Field0F8;
        int m_Field0FC;
        char m_Field100;
        char field_101;
        char field_102;
        char field_103;
        int m_Field104;
        float m_Field108;
        boost::shared_ptr<void> m_spParamCueName;
        char m_Field114;
        float m_Field118;
    };

    class CObjSound3D : public CObjSound
    {
    public:
        CObjSound3D(const bb_null_ctor& nil) : CObjSound(nil)
        {
        }

        CObjSound3D() : CObjSound3D(bb_null_ctor{})
        {
            fpCObjSound3DCtor(this);
        }

        BB_OVERRIDE_FUNCTION_PTR(void, CObjSound, UpdateParallel, 0x10050D0,
                                 (const Hedgehog::Universe::SUpdateInfo&, in_rUpdateInfo))
        BB_OVERRIDE_FUNCTION_PTR(bool, CMessageActor, ProcessMessage, 0x1005C50,
                                 (Hedgehog::Universe::Message&, in_rMsg), (bool, in_Flag))

        Hedgehog::Math::CVector m_Field120;
        Hedgehog::Math::CVector m_Field130;
    };

    namespace SoundPoint
    {
        class CObjSoundPoint;
        //MAKE_BASIC_OPTIMIZED_CTOR(CObjSoundPoint, 0x0114E750, esi)

        static constexpr int pFuncSoundPt = 0x0114E750;

        static void __declspec(naked) fCObjSoundPointCtor()
        {
            __asm { mov esi, ecx }
            __asm { jmp[pFuncSoundPt] }
        }


        static void fCObjSoundPointCtor2(CObjSoundPoint* This)
        {
            __asm
                {
                mov esi, This
                call [pFuncSoundPt]
                }
        }

        static BB_FUNCTION_PTR(void, __thiscall, fpCObjSoundPointCtor, fCObjSoundPointCtor, CObjSoundPoint* This);

        class CObjSoundPoint : public CObjSound3D
        {
        public:
            CObjSoundPoint(const bb_null_ctor& nil) : CObjSound3D(nil)
            {
            }

            CObjSoundPoint() : CObjSoundPoint(bb_null_ctor{})
            {
                //fpCObjSoundPointCtor(this);
                fCObjSoundPointCtor2(this);
            }

            BB_OVERRIDE_FUNCTION_PTR(void, CObjSoundPoint, UpdateParallel, 0x114E380,
                                     (const Hedgehog::Universe::SUpdateInfo&, in_rUpdateInfo))
            BB_OVERRIDE_FUNCTION_PTR(void, CObjSoundPoint, AddCallback, 0x114E3C0,
                                     (const Hedgehog::Base::THolder<CWorld>&, in_rWorldHolder),
                                     (Sonic::CGameDocument*, in_pGameDocument),
                                     (const boost::shared_ptr<Hedgehog::Database::CDatabase>&, in_spDatabase))
            BB_OVERRIDE_FUNCTION_PTR(bool, CMessageActor, ProcessMessage, 0x114E940,
                                     (Hedgehog::Universe::Message&, in_rMsg), (bool, in_Flag))
            //BB_OVERRIDE_FUNCTION_PTR(void, CSetObjectListener, InitializeEditParam, 0x114E500, (CEditParam&, in_rEditParam))
            void InitializeEditParam(CEditParam& in_rEditParam) override
            {
                return ((void(__thiscall*)(CSetObjectListener*, CEditParam&))(0x114E500))(this, in_rEditParam);
            }

            int m_Field140;
            int m_Field144;
            float m_Field148;
            float m_Field14C;
        };

        BB_ASSERT_SIZEOF(CObjSoundPoint, 0x150);
        //BB_SET_OBJECT_MAKE_HOOK(CObjSoundPoint)
    }
}
