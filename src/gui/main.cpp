#include "main.hpp"

#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMenuBar>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTextEdit>
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

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <map>
#include <chrono>
#include <thread>

#include "Nodes/Node.hpp"
#include "Nodes/NodeManager.hpp"
#include "NodeRegistry.hpp"
#include "util/Logger.hpp"

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

NAV::NodeManager nodeManager;
std::shared_ptr<QtNodes::DataModelRegistry> registryAll;
std::shared_ptr<QtNodes::DataModelRegistry> registryRealTime;
std::shared_ptr<QtNodes::DataModelRegistry> registryPostProcessing;
QAction* rtpAction;

std::string delimiter = " _,_ ";

void addTypeConverter(std::shared_ptr<DataModelRegistry> registry, std::string_view child, std::string_view root)
{
    LOG_TRACE("called");
    if (nodeManager.registeredNodeDataTypes().count(child))
    {
        const auto& parents = nodeManager.registeredNodeDataTypes().find(child)->second.parents;

        for (const auto& parent : parents)
        {
            std::cout << "TypeConvert: " << root << " --> " << parent << std::endl;
            registry->registerTypeConverter(std::make_pair(NodeDataType{ QString::fromStdString(std::string(root)), QString::fromStdString(std::string(root)) },
                                                           NodeDataType{ QString::fromStdString(std::string(parent)), QString::fromStdString(std::string(parent)) }),
                                            TypeConverter{ Converter() });
            addTypeConverter(registry, parent, root);
        }
    }
}

static std::shared_ptr<DataModelRegistry> registerDataModels(NAV::Node::NodeContext compat)
{
    LOG_TRACE("called for context: {}", compat == NAV::Node::NodeContext::REAL_TIME
                                            ? "Real Time"
                                            : (compat == NAV::Node::NodeContext::POST_PROCESSING
                                                   ? "Post Processing"
                                                   : "All"));

    auto registry = std::make_shared<DataModelRegistry>();

    for (const auto& [type, nodeInfo] : nodeManager.registeredNodeTypes())
    {
        if (nodeInfo.constructorEmpty()->context() == compat || nodeInfo.constructorEmpty()->context() == NAV::Node::NodeContext::ALL || compat == NAV::Node::NodeContext::ALL)
        {
            auto node = nodeInfo.constructorEmpty();
            registry->registerModelTemplate<NodeModel>(QString::fromStdString(std::string(type)),
                                                       QString::fromStdString(std::string(node->category())));
        }
    }

    for (const auto& [type, nodeDataInfo] : nodeManager.registeredNodeDataTypes())
    {
        addTypeConverter(registry, type, type);
    }

    return registry;
}

void exportConfigForLayout(QFormLayout* layout, std::string& comment, std::string& config)
{
    LOG_DEBUG("Items in QFormLayout: {}", layout->rowCount());

    for (int i = 0; i < layout->rowCount(); i++)
    {
        QWidget* widget = layout->itemAt(i, QFormLayout::ItemRole::FieldRole)->widget();

        if (widget->layout()
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_MULTI
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_MULTI
            && widget->property("type").toUInt() != NAV::Node::ConfigOptionType::CONFIG_STRING_BOX)
        {
            exportConfigForLayout(static_cast<QFormLayout*>(widget->layout()), comment, config);
        }
        else
        {
            if (widget->objectName().startsWith("Port "))
                continue;

            LOG_DEBUG("Exporting {}", widget->objectName().toStdString());
            std::string text;
            if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_BOOL)
                text = std::to_string(static_cast<QCheckBox*>(widget)->isChecked());
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_INT)
                text = std::to_string(static_cast<QSpinBox*>(widget)->value());
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_N_INPUT_PORTS)
                text = std::to_string(static_cast<QSpinBox*>(widget)->value());
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_FLOAT)
                text = std::to_string(static_cast<QDoubleSpinBox*>(widget)->value());
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_STRING)
                text = static_cast<QLineEdit*>(widget)->text().replace("\n", "\\n").replace("#", "[hash]").toStdString();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_STRING_BOX)
            {
                auto groupBox = static_cast<QGroupBox*>(widget);
                auto layout = static_cast<QVBoxLayout*>(groupBox->layout());

                QTextEdit* textEdit = static_cast<QTextEdit*>(layout->itemAt(0)->widget());

                text = textEdit->toPlainText().replace("\n", "\\n").replace("#", "[hash]").toStdString();
            }
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST)
                text = static_cast<QComboBox*>(widget)->currentText().toStdString();
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST_MULTI)
            {
                auto groupBox = static_cast<QGroupBox*>(widget);
                auto layout = static_cast<QFormLayout*>(groupBox->layout());

                for (int j = 1; j < layout->rowCount(); j++)
                {
                    QComboBox* list = static_cast<QComboBox*>(layout->itemAt(j, QFormLayout::ItemRole::FieldRole)->widget());

                    std::string toAdd = list->currentText().toStdString();

                    if (text.find(toAdd) == std::string::npos)
                        text += (!text.empty() ? ";" : "") + toAdd;
                }
            }
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_LIST_LIST_MULTI)
            {
                auto gridGroupBox = static_cast<QGroupBox*>(widget);
                auto layout = static_cast<QGridLayout*>(gridGroupBox->layout());

                for (int j = 2; j < layout->count() / 2; j++)
                {
                    QComboBox* xlist = static_cast<QComboBox*>(layout->itemAtPosition(j, 0)->widget());
                    QComboBox* ylist = static_cast<QComboBox*>(layout->itemAtPosition(j, 1)->widget());

                    std::string toAdd = xlist->currentText().toStdString() + "|" + ylist->currentText().toStdString();

                    if (text.find(toAdd) == std::string::npos)
                        text += (!text.empty() ? ";" : "") + toAdd;
                }
            }
            else if (widget->property("type").toUInt() == NAV::Node::ConfigOptionType::CONFIG_MAP_INT)
                text = widget->property("key").toString().toStdString() + ", " + std::to_string(static_cast<QSpinBox*>(widget)->value());

            std::string type = widget->objectName().toStdString();

            std::replace(type.begin(), type.end(), '\n', ' ');

            config += delimiter + "\"" + type + "\" = \"" + text + "\"";

            // comment += ", " + type;
            // for (int i = 0; i < static_cast<int>(type.size()) - static_cast<int>(text.size()); i++)
            //     config += " ";
            // for (int i = 0; i < static_cast<int>(text.size()) - static_cast<int>(type.size()); i++)
            //     comment += " ";
        }
    }
}

void exportConfig()
{
    LOG_TRACE("called");

    std::ofstream filestream;
    filestream.open("config-dataflow.ini", std::ios_base::trunc);

    for (auto node : scene->allNodes())
    {
        auto nodeModel = static_cast<NodeModel*>(node->nodeDataModel());
        std::string comment = "#      Type";
        for (int i = 0; i < nodeModel->name().length() - 4; i++)
            comment += " ";
        comment += delimiter + "Name";
        for (int i = 0; i < node->id().toString().length() - 4; i++)
            comment += " ";

        std::string config = "node = " + nodeModel->name().toStdString() + delimiter + node->id().toString().toStdString();

        exportConfigForLayout(nodeModel->getMainLayout(), comment, config);

        filestream << comment << std::endl;
        filestream << config << std::endl;
    }

    filestream << std::endl
               << "#      source, data type, target" << std::endl;
    for (auto connection : scene->connections())
    {
        auto source = connection.second->getNode(QtNodes::PortType::Out);
        auto sourcePort = connection.second->getPortIndex(QtNodes::PortType::Out);
        // const auto& sourceNodeInfo = nodeManager.registeredNodeTypes().find(source->nodeDataModel()->name().toStdString())->second;
        auto target = connection.second->getNode(QtNodes::PortType::In);
        auto targetPort = connection.second->getPortIndex(QtNodes::PortType::In);

        filestream << "link = "
                   << source->id().toString().toStdString() << ", "
                   << sourcePort << ", "
                   << targetPort << ", "
                   //    << sourceNodeInfo.constructorEmpty()->dataType(NAV::Node::PortType::Out, static_cast<uint8_t>(sourcePort)) << ", "
                   << target->id().toString().toStdString() << '\n';
    }

    filestream.close();
}

bool save()
{
    LOG_TRACE("called");
    if (fileName.isEmpty())
    {
        fileName = scene->save();
    }
    else
    {
        scene->saveAs(fileName);
    }

    if (!fileName.isEmpty())
    {
        exportConfig();
        return true;
    }

    return false;
}

bool saveAs()
{
    LOG_TRACE("called");
    fileName = scene->save();

    if (!fileName.isEmpty())
    {
        exportConfig();
        return true;
    }

    return false;
}

void load()
{
    LOG_TRACE("called");
    rtpAction->setText("All Nodes");
    scene->setRegistry(registryAll);
    registrySelected = 0;

    auto tmp = scene->load();
    if (!tmp.isEmpty())
        fileName = tmp;
}

void run()
{
    LOG_TRACE("called");
    if (scene->nodes().size() > 0)
    {
        exportConfig();
        system("pkill -SIGUSR1 -x navsos");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        system("./bin/Debug/navsos -f config.ini config-dataflow.ini &");
    }
}

void clearScene()
{
    LOG_TRACE("called");
    scene->clearScene();
    fileName = "";
}

void changeRegistry()
{
    LOG_TRACE("called");
    if (registrySelected == 0)
    {
        size_t incompNodes = 0;
        for (auto node : scene->allNodes())
        {
            auto nodeModel = static_cast<NodeModel*>(node->nodeDataModel());

            const auto& nodeInfo = nodeManager.registeredNodeTypes().find(nodeModel->name().toStdString())->second;

            if (nodeInfo.constructorEmpty()->context() != NAV::Node::NodeContext::REAL_TIME
                && nodeInfo.constructorEmpty()->context() != NAV::Node::NodeContext::ALL)
            {
                incompNodes++;
                break;
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

            const auto& nodeInfo = nodeManager.registeredNodeTypes().find(nodeModel->name().toStdString())->second;

            if (nodeInfo.constructorEmpty()->context() != NAV::Node::NodeContext::POST_PROCESSING
                && nodeInfo.constructorEmpty()->context() != NAV::Node::NodeContext::ALL)
            {
                incompNodes++;
                break;
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
    Logger logger("logs/navsos-gui.log");

    QApplication app(argc, argv);

    setStyle();

    QWidget mainWidget;

    QMenuBar* menuBarLeft = new QMenuBar(&mainWidget);
    QAction* runAction = menuBarLeft->addAction("Run NavSoS");
    QAction* loadAction = menuBarLeft->addAction("Load");
    QAction* saveAction = menuBarLeft->addAction("Save As");
    QAction* clearAction = menuBarLeft->addAction("Clear");
    QAction* clearExit = menuBarLeft->addAction("Exit");

    QMenuBar* menuBarRight = new QMenuBar(&mainWidget);
    rtpAction = menuBarRight->addAction("All Nodes");

    QGridLayout* l = new QGridLayout(&mainWidget);

    // Register all Node Types which are available to the program
    NAV::NodeRegistry::registerNodeTypes(nodeManager);

    // Register all Node Data Types which are available to the program
    NAV::NodeRegistry::registerNodeDataTypes(nodeManager);

    registryAll = registerDataModels(NAV::Node::NodeContext::ALL);
    registryRealTime = registerDataModels(NAV::Node::NodeContext::REAL_TIME);
    registryPostProcessing = registerDataModels(NAV::Node::NodeContext::POST_PROCESSING);

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
