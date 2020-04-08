#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>

#include <nodes/Node>
#include <nodes/Connection>
#include <nodes/NodeData>
#include <nodes/FlowScene>
#include <nodes/FlowView>
#include <nodes/DataModelRegistry>
#include <nodes/ConnectionStyle>
#include <nodes/TypeConverter>

#include "models.hpp"

#include <iostream>

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

static std::shared_ptr<DataModelRegistry>
    registerDataModels()
{
    auto ret = std::make_shared<DataModelRegistry>();

    ret->registerModelTemplate<NodeDataTemplate>("VectorNavSensor", "Category");
    ret->registerModelTemplate<NodeDataTemplate>("UbloxSensor", "Category");
    ret->registerModelTemplate<NodeDataTemplate>("VectorNavDataLogger", "Category");
    ret->registerModelTemplate<NodeDataTemplate>("Integrator", "Category");

    ret->registerTypeConverter(std::make_pair(NodeDataType{ "VectorNavObs", "VectorNavObs" },
                                              NodeDataType{ "InsObs", "InsObs" }),
                               TypeConverter{ Converter() });

    return ret;
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

void exportConfig()
{
    for (auto node : scene->allNodes())
    {
        std::cout << node->nodeDataModel()->name().toStdString() << ": "
                  << node->id().toString().toStdString() << std::endl;
        for (int i = 0; i < 5; i++)
        {
            std::cout << node->nodeDataModel()->embeddedWidget()->children().at(i)->objectName().toStdString() << std::endl;
        }
    }

    for (auto connection : scene->connections())
    {
        std::cout << "(" << connection.second->getPortIndex(QtNodes::PortType::Out) << ") "
                  << connection.second->getNode(QtNodes::PortType::Out)->id().toString().toStdString()
                  << " ==> "
                  << "(" << connection.second->getPortIndex(QtNodes::PortType::In) << ") "
                  << connection.second->getNode(QtNodes::PortType::In)->id().toString().toStdString()
                  << std::endl;
    }
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    setStyle();

    QWidget mainWidget;

    auto menuBar = new QMenuBar();

    auto saveAction = menuBar->addAction("Save");
    auto loadAction = menuBar->addAction("Load");
    auto clearAction = menuBar->addAction("Clear");
    auto clearExit = menuBar->addAction("Exit");
    auto exportAction = menuBar->addAction("Export Config");

    QVBoxLayout* l = new QVBoxLayout(&mainWidget);

    l->addWidget(menuBar);
    scene = new FlowScene(registerDataModels(), &mainWidget);
    l->addWidget(new FlowView(scene));
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(0);

    QObject::connect(exportAction, &QAction::triggered,
                     scene, &exportConfig);

    QObject::connect(saveAction, &QAction::triggered,
                     scene, &FlowScene::save);

    QObject::connect(loadAction, &QAction::triggered,
                     scene, &FlowScene::load);

    QObject::connect(clearAction, &QAction::triggered,
                     scene, &FlowScene::clearScene);

    QObject::connect(clearExit, &QAction::triggered,
                     scene, &app.exit);

    mainWidget.setWindowTitle("Node-based flow editor");
    mainWidget.resize(800, 600);
    mainWidget.showNormal();

    return app.exec();
}
