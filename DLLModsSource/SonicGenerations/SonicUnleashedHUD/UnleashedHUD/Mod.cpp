extern "C" __declspec(dllexport) void Init(const char* path)
{
	Configuration::Read();

	ArchiveTreePatcher::Install();
	HudSonicStage::Install();
	Patches::Install();
}