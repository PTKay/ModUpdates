/// <summary>
/// Hooked function that returns Sonic Unleashed's icon - following the same pattern as LoadIconA.
/// </summary>
HICON __stdcall LoadSonicUnleashedIcon(HINSTANCE hInstance, LPCSTR lpIconName)
{
	return LoadIconA(DllMain::handle, (LPCSTR)IDI_ICON);
}

void Patches::Install()
{
	if (Configuration::windowTitle)
	{
		// Use custom game window title.
		const char* title = (Configuration::windowTitleRegion == Configuration::WindowTitleRegion::English ? "SONIC UNLEASHED" : "Sonic World Adventure");
		WRITE_STATIC_MEMORY(0x1606C50, title + '\0', strlen(title) + 1);

		// Use Sonic Unleashed's icon for the window.
		WRITE_CALL(0xE7B843, &LoadSonicUnleashedIcon);
		WRITE_NOP(0xE7B848, 1);
	}
}