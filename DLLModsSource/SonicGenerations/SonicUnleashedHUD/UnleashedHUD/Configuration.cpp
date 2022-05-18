bool Configuration::windowTitle = true;
Configuration::WindowTitleRegion Configuration::windowTitleRegion = WindowTitleRegion::English;
Configuration::ButtonType Configuration::buttonType = ButtonType::X360;

void Configuration::Read()
{
	INIReader reader(INI_FILE);

	// Appearance
	windowTitle		  = reader.GetBoolean("Appearance", "windowTitle", windowTitle);
	windowTitleRegion = (WindowTitleRegion)reader.GetInteger("Appearance", "windowTitleRegion", windowTitleRegion);
	buttonType		  = (ButtonType)reader.GetInteger("Appearance", "buttonType", buttonType);
}