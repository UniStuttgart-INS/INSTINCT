#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
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

static std::shared_ptr<DataModelRegistry> registerDataModels(NAV::NodeInterface::NodeContext compat)
{
    auto registry = std::make_shared<DataModelRegistry>();

    for (auto it = NAV::nodeInterfaces.cbegin(); it != NAV::nodeInterfaces.cend(); it++)
        if (it->second.nodeCompat == compat || it->second.nodeCompat == NAV::NodeInterface::NodeContext::ALL || compat == NAV::NodeInterface::NodeContext::ALL)
            registry->registerModelTemplate<NodeModel>(QString::fromStdString(it->first), QString::fromStdString(it->second.category));

    for (auto child = NAV::inheritance.cbegin(); child != NAV::inheritance.cend(); child++)
        addTypeConverter(registry, child->first, child->first);

    return registry;
}

static void setStyle()
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
int registrySelected = 0;
std::shared_ptr<QtNodes::DataModelRegistry> registryAll;
std::shared_ptr<QtNodes::DataModelRegistry> registryRealTime;
std::shared_ptr<QtNodes::DataModelRegistry> registryPostProcessing;
QAction* rtpAction;

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
            else if (nodeModel->widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_LIST_LIST_INT)
            {
                auto gridGroupBox = static_cast<QGroupBox*>(nodeModel->widgets.at(i));
                auto layout = static_cast<QGridLayout*>(gridGroupBox->layout());

                for (int j = 1; j < layout->rowCount(); j++)
                {
                    QComboBox* xlist = static_cast<QComboBox*>(layout->itemAtPosition(j, 0)->widget());
                    QComboBox* ylist = static_cast<QComboBox*>(layout->itemAtPosition(j, 1)->widget());
                    QSpinBox* spinBox = static_cast<QSpinBox*>(layout->itemAtPosition(j, 2)->widget());

                    if (spinBox->value() != -1)
                    {
                        std::string toAdd = xlist->currentText().toStdString() + ", " + ylist->currentText().toStdString() + ", " + std::to_string(spinBox->value());

                        if (text.find(toAdd) == std::string::npos)
                            text += (!text.empty() ? ", " : "") + toAdd;
                    }
                }
            }
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
    rtpAction->setText("All Nodes");
    scene->setRegistry(registryAll);
    registrySelected = 0;

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

void changeRegistry()
{
    if (registrySelected == 0)
    {
        size_t incompNodes = 0;
        for (auto node : scene->allNodes())
        {
            auto nodeModel = static_cast<NodeModel*>(node->nodeDataModel());
            for (auto it = NAV::nodeInterfaces.cbegin(); it != NAV::nodeInterfaces.cend(); it++)
            {
                if (nodeModel->name().toStdString() == it->first)
                {
                    if (it->second.nodeCompat != NAV::NodeInterface::NodeContext::REAL_TIME
                        && it->second.nodeCompat != NAV::NodeInterface::NodeContext::ALL)
                    {
                        incompNodes++;
                        break;
                    }
                }
            }
        }
        if (incompNodes == 0)
        {
            rtpAction->setText("Real-Time");
            scene->setRegistry(registryRealTime);
            registrySelected++;
            return;
        }
        else
        {
            std::cout << "Can't switch to Real-Time Context, because there are " << incompNodes << " incompatible Nodes." << std::endl;
            registrySelected++;
        }
    }

    if (registrySelected == 1)
    {
        size_t incompNodes = 0;
        for (auto node : scene->allNodes())
        {
            auto nodeModel = static_cast<NodeModel*>(node->nodeDataModel());
            for (auto it = NAV::nodeInterfaces.cbegin(); it != NAV::nodeInterfaces.cend(); it++)
            {
                if (nodeModel->name().toStdString() == it->first)
                {
                    if (it->second.nodeCompat != NAV::NodeInterface::NodeContext::POST_PROCESSING
                        && it->second.nodeCompat != NAV::NodeInterface::NodeContext::ALL)
                    {
                        incompNodes++;
                        break;
                    }
                }
            }
        }

        if (incompNodes == 0)
        {
            rtpAction->setText("Post Processing");
            scene->setRegistry(registryPostProcessing);
            registrySelected++;
            return;
        }
        else
        {
            std::cout << "Can't switch to Post Processing Context, because there are " << incompNodes << " incompatible Nodes." << std::endl;
            registrySelected++;
        }
    }

    if (registrySelected == 2)
    {
        rtpAction->setText("All Nodes");
        scene->setRegistry(registryAll);
        registrySelected = 0;
        return;
    }
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    setStyle();

    QWidget mainWidget;

    QMenuBar* menuBarLeft = new QMenuBar();
    QAction* runAction = menuBarLeft->addAction("Run NavSoS");
    QAction* loadAction = menuBarLeft->addAction("Load");
    QAction* saveAction = menuBarLeft->addAction("Save As");
    QAction* clearAction = menuBarLeft->addAction("Clear");
    QAction* clearExit = menuBarLeft->addAction("Exit");

    QMenuBar* menuBarRight = new QMenuBar();
    rtpAction = menuBarRight->addAction("All Nodes");

    QGridLayout* l = new QGridLayout(&mainWidget);

    registryAll = registerDataModels(NAV::NodeInterface::NodeContext::ALL);
    registryRealTime = registerDataModels(NAV::NodeInterface::NodeContext::REAL_TIME);
    registryPostProcessing = registerDataModels(NAV::NodeInterface::NodeContext::POST_PROCESSING);

    l->addWidget(menuBarLeft, 0, 0);
    l->addWidget(menuBarRight, 0, 1);
    l->setAlignment(menuBarLeft, Qt::AlignLeft);
    l->setAlignment(menuBarRight, Qt::AlignRight);

    scene = new FlowScene(registryAll, &mainWidget);
    l->addWidget(new FlowView(scene), 1, 0, 1, 2);
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

    QObject::connect(rtpAction, &QAction::triggered,
                     scene, &changeRegistry);

    mainWidget.setWindowTitle("NavSoS - Navigation Software Stuttgart (Institut of Navigation)");
    mainWidget.resize(800, 600);
    mainWidget.showNormal();

    scene->load("flow/Default.flow");

    return app.exec();
}
