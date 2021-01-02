#include "gui/NodeEditorApplication.hpp"

#include "util/Logger.hpp"
#include <iostream>

#include "NodeRegistry.hpp"

int Main(int argc, const char* argv[])
{
    try
    {
        Logger logger("logs/navsos.log");

        // Register all Node Types which are available to the program
        NAV::NodeRegistry::registerNodeTypes();

        // Register all Node Data Types which are available to the program
        NAV::NodeRegistry::registerNodeDataTypes();

        NAV::gui::NodeEditorApplication app("NavSoS - Navigation Software Stuttgart (Institute of Navigation)", "NavSoS.ini", argc, argv);

        if (app.Create())
        {
            return app.Run();
        }

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Critical Event occurred: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}