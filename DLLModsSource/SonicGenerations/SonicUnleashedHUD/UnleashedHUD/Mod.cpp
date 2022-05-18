extern "C" __declspec(dllexport) void Init(const char* path)
{
	Configuration::Read();

	ControllerPatcher::Install();
	HudSonicStage::Install();
	Patches::Install();
}