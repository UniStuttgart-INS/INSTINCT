#pragma once

#include <QtCore/QObject>

#include <nodes/NodeData>
#include <nodes/NodeDataModel>

#include <memory>
#include <vector>

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::PortType;
using QtNodes::PortIndex;

/// The model dictates the number of inputs and outputs for the Node
class NodeModel : public NodeDataModel
{
    Q_OBJECT

  public:
    NodeModel();

    NodeModel(QString const& name);

    // Do not delete objects here, as qt handles it himself
    virtual ~NodeModel();

  public:
    QString caption() const override;

    QString name() const override;

  public:
    unsigned int nPorts(PortType portType) const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;

    void restore(QJsonObject const& p) override;

    QWidget* embeddedWidget() override;

    std::shared_ptr<NodeData> outData(PortIndex) override;

    void setInData(std::shared_ptr<NodeData>, int) override;

  private:
    QString const _name = "Template";
    QWidget* _mainWidget;

  public:
    std::vector<QWidget*> widgets;
};
