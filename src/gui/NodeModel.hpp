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

    void saveLayoutItems(QFormLayout* layout, QJsonObject& modelJson) const;

    QJsonObject save() const override;

    void restoreLayoutItems(QFormLayout* layout, QJsonObject const& p);

    void restore(QJsonObject const& p) override;

    QWidget* embeddedWidget() override;

    QFormLayout* getMainLayout() { return _mainLayout; }

  private:
    QString const _name = "Template";
    QWidget* _mainWidget;
    QFormLayout* _mainLayout;

    void addGuiElementForConfig(const NAV::Node::ConfigOptions& config,
                                const std::vector<NAV::Node::ConfigOptions>& guiConfigs,
                                const size_t configPosition,
                                QFormLayout* _layout,
                                int layoutInsertPosition,
                                QString prefix = "");

    void addRepeatedConfigGroupBox(const std::vector<NAV::Node::ConfigOptions>& guiConfigs,
                                   QFormLayout* _layout,
                                   size_t portNumber,
                                   size_t configRepeatedStart,
                                   size_t configRepeatedNumber);

    void removeRepeatedConfigGroupBox(QSpinBox* inputSpinBox);

    void addListListIntRow(std::vector<std::variant<std::string, NAV::Node::ConfigOptionsBase>> config, int row, QGridLayout* layout, QGroupBox* gridGroupBox, QFormLayout* formLayout);

    void updateView(QSpinBox* inputSpinBox);

    void updateVariants(const std::vector<NAV::Node::ConfigOptions>& guiConfigs, const size_t configPosition, QFormLayout* layout, int layoutPosition);

    void clearLayout(QFormLayout* layout);
};
