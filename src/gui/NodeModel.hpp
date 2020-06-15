#pragma once

#include <QtCore/QObject>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QSpinBox>

#include <nodes/NodeData>
#include <nodes/NodeDataModel>

#include <memory>
#include <vector>

#include "main.hpp"

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
    explicit NodeModel(QString const& name);

    QString caption() const override;

    QString name() const override;

    unsigned int nPorts(PortType portType) const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;

    void restore(QJsonObject const& p) override;

    QWidget* embeddedWidget() override;

    std::vector<QWidget*> widgets;

  private:
    QString const _name = "Template";
    QWidget* _mainWidget;
    QFormLayout* _mainLayout;

    void addGuiElementForConfig(const NAV::Node::ConfigOptions& config, QFormLayout* _layout, QString prefix = "");

    void addRepeatedConfigGroupBox(const std::vector<NAV::Node::ConfigOptions>& guiConfigs,
                                   QFormLayout* _layout,
                                   size_t portNumber,
                                   size_t configRepeatedStart,
                                   size_t configRepeatedNumber);

    void removeRepeatedConfigGroupBox(QSpinBox* inputSpinBox);

    void addListListIntRow(std::vector<std::string> config, int row, QGridLayout* layout, QGroupBox* gridGroupBox, QFormLayout* formLayout);

    void updateView(QSpinBox* inputSpinBox);

    void clearLayout(QFormLayout* layout);
};
