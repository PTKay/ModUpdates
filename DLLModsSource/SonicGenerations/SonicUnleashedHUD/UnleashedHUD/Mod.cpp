extern "C" __declspec(dllexport) void Init(const char* path)
{
	Configuration::Read();

	HudSonicStage::Install();
	Patches::Install();
	ArchiveTreePatcher::Install();
}