#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QComboBox>

#include <nodes/Node>
#include <nodes/Connection>
#include <nodes/NodeData>
#include <nodes/FlowScene>
#include <nodes/FlowView>
#include <nodes/DataModelRegistry>
#include <nodes/ConnectionStyle>
#include <nodes/TypeConverter>

#include "NodeModel.hpp"
#define GUI
#include <../NodeInterface.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

using QtNodes::DataModelRegistry;
using QtNodes::FlowScene;
using QtNodes::FlowView;
using QtNodes::ConnectionStyle;
using QtNodes::TypeConverter;
using QtNodes::TypeConverterId;

class Converter
{
  public:
    std::shared_ptr<NodeData>
        operator()(std::shared_ptr<NodeData> /*data*/)
    {
        return nullptr;
    }
};

void addTypeConverter(std::shared_ptr<DataModelRegistry> registry, std::string child, std::string root)
{
    if (NAV::inheritance.count(root))
    {
        for (auto parent = NAV::inheritance.at(root).cbegin(); parent != NAV::inheritance.at(root).cend(); parent++)
        {
            std::cout << "TypeConvert: " << child << " --> " << *parent << std::endl;
            registry->registerTypeConverter(std::make_pair(NodeDataType{ QString::fromStdString(child), QString::fromStdString(child) },
                                                           NodeDataType{ QString::fromStdString(*parent), QString::fromStdString(*parent) }),
                                            TypeConverter{ Converter() });
            addTypeConverter(registry, child, *parent);
        }
    }
}

static std::shared_ptr<DataModelRegistry>
    registerDataModels()
{
    auto registry = std::make_shared<DataModelRegistry>();

    for (auto it = NAV::nodeInterfaces.cbegin(); it != NAV::nodeInterfaces.cend(); it++)
        registry->registerModelTemplate<NodeModel>(QString::fromStdString(it->first), QString::fromStdString(it->second.category));

    for (auto child = NAV::inheritance.cbegin(); child != NAV::inheritance.cend(); child++)
        addTypeConverter(registry, child->first, child->first);

    return registry;
}

static void
    setStyle()
{
    ConnectionStyle::setConnectionStyle(
        R"(
  {
    "ConnectionStyle": {
      "UseDataDefinedColors": true
    }
  }
  )");
}

//------------------------------------------------------------------------------
FlowScene* scene;
QString fileName;

void exportConfig()
{
    std::ofstream filestream;
    filestream.open("config-dataflow.ini", std::ios_base::trunc);

    for (auto node : scene->allNodes())
    {
        auto nodeModel = static_cast<NodeModel*>(node->nodeDataModel());
        std::string comment = "#      Type";
        for (int i = 0; i < nodeModel->name().length() - 4; i++)
            comment += " ";
        comment += ", Name";
        for (int i = 0; i < node->id().toString().length() - 4; i++)
            comment += " ";

        std::string config = "node = " + nodeModel->name().toStdString() + ", " + node->id().toString().toStdString();
        for (size_t i = 0; i < nodeModel->widgets.size(); i++)
        {
            std::string text;
            if (nodeModel->widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_BOOL)
                text = std::to_string(static_cast<QCheckBox*>(nodeModel->widgets.at(i))->isChecked());
            else if (nodeModel->widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_INT)
                text = std::to_string(static_cast<QSpinBox*>(nodeModel->widgets.at(i))->value());
            else if (nodeModel->widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_FLOAT)
                text = std::to_string(static_cast<QDoubleSpinBox*>(nodeModel->widgets.at(i))->value());
            else if (nodeModel->widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_STRING)
                text = static_cast<QLineEdit*>(nodeModel->widgets.at(i))->text().toStdString();
            else if (nodeModel->widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_LIST)
                text = static_cast<QComboBox*>(nodeModel->widgets.at(i))->currentText().toStdString();
            else if (nodeModel->widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_MAP_INT)
                text = nodeModel->widgets.at(i)->property("key").toString().toStdString() + ", " + std::to_string(static_cast<QSpinBox*>(nodeModel->widgets.at(i))->value());

            std::string type = nodeModel->widgets.at(i)->objectName().toStdString();

            std::replace(type.begin(), type.end(), '\n', ' ');

            comment += ", " + type;
            config += ", " + text;

            for (int i = 0; i < static_cast<int>(type.size()) - static_cast<int>(text.size()); i++)
                config += " ";
            for (int i = 0; i < static_cast<int>(text.size()) - static_cast<int>(type.size()); i++)
                comment += " ";
        }

        filestream << comment << std::endl;
        filestream << config << std::endl;
    }

    filestream << std::endl
               << "#      source, data type, target" << std::endl;
    for (auto connection : scene->connections())
    {
        auto source = connection.second->getNode(QtNodes::PortType::Out);
        auto sourcePort = connection.second->getPortIndex(QtNodes::PortType::Out);
        auto& sourceNodeInterface = NAV::nodeInterfaces.at(source->nodeDataModel()->name().toStdString());
        auto target = connection.second->getNode(QtNodes::PortType::In);

        filestream << "link = "
                   << source->id().toString().toStdString() << ", "
                   << sourceNodeInterface.out.at(static_cast<size_t>(sourcePort)) << ", "
                   << target->id().toString().toStdString() << std::endl;
    }

    filestream.close();
}

bool save()
{
    if (fileName.isEmpty())
        fileName = scene->save();
    else
        scene->saveAs(fileName);

    if (!fileName.isEmpty())
    {
        exportConfig();
        return true;
    }
    else
        return false;
}

bool saveAs()
{
    fileName = scene->save();

    if (!fileName.isEmpty())
    {
        exportConfig();
        return true;
    }
    else
        return false;
}

void load()
{
    auto tmp = scene->load();
    if (!tmp.isEmpty())
        fileName = tmp;
}

void run()
{
    if (scene->nodes().size() > 0)
    {
        exportConfig();
        system("./bin/Debug/navsos -f config.ini config-dataflow.ini");
    }
}

void clearScene()
{
    scene->clearScene();
    fileName = "";
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    setStyle();

    QWidget mainWidget;

    auto menuBar = new QMenuBar();

    auto runAction = menuBar->addAction("Run NavSoS");
    auto loadAction = menuBar->addAction("Load");
    auto saveAction = menuBar->addAction("Save As");
    auto clearAction = menuBar->addAction("Clear");
    auto clearExit = menuBar->addAction("Exit");

    QVBoxLayout* l = new QVBoxLayout(&mainWidget);

    l->addWidget(menuBar);
    scene = new FlowScene(registerDataModels(), &mainWidget);
    l->addWidget(new FlowView(scene));
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(0);

    QObject::connect(runAction, &QAction::triggered,
                     scene, &run);

    QObject::connect(saveAction, &QAction::triggered,
                     scene, &saveAs);

    QObject::connect(loadAction, &QAction::triggered,
                     scene, &load);

    QObject::connect(clearAction, &QAction::triggered,
                     scene, &clearScene);

    QObject::connect(clearExit, &QAction::triggered,
                     scene, &app.exit);

    mainWidget.setWindowTitle("NavSoS - Navigation Software Stuttgart (Institut of Navigation)");
    mainWidget.resize(800, 600);
    mainWidget.showNormal();

    scene->load("flow/Default.flow");

    return app.exec();
}
