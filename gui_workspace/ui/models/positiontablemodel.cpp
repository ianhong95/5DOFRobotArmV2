#include "positiontablemodel.h"

#include <QAbstractTableModel>
#include <QVector>
#include <QString>
#include <QVariant>

PositionTableModel::PositionTableModel(QObject *parent) {

}

Qt::ItemFlags PositionTableModel::flags(const QModelIndex &index) const {
    if (!index.isValid()) {
        return Qt::NoItemFlags;
    }

    return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
}

bool PositionTableModel::setData(const QModelIndex &index, const QVariant &value, int role) {
    if (!index.isValid() || role != Qt::EditRole)
        return false;

    SavedPosition &positionEntry = savedPositions[index.row()];

    // Value is the NEW value that you typed into the cell. Break out of switch as soon as one case is evaluated.
    switch (index.column()) {
        case 0:
            positionEntry.index = value.toInt();
            break;
        case 1:
            positionEntry.alias = value.toString();
            break;
        case 2:
            positionEntry.x = value.toFloat();
            break;
        case 3:
            positionEntry.y = value.toFloat();
            break;
        case 4:
            positionEntry.z = value.toFloat();
            break;
        default:
            return false;
    }

    emit dataChanged(index, index, {Qt::EditRole});     // Qt's internal signal

    return true;

    // Add an emit handler to update server
}

// Implement the magic functions

int PositionTableModel::rowCount(const QModelIndex &parent) const {
    if (parent.isValid()) {
        return 0;
    }
    return savedPositions.size();
}

int PositionTableModel::columnCount(const QModelIndex &parent) const {
    return 5;
}

QVariant PositionTableModel::data(const QModelIndex &index, int role) const {
    SavedPosition positionEntry = savedPositions[index.row()];

    /* The way the data() method works is it is called multiple times by Qt, each time with a different role.
     * This initializes the data with all possible properties (showing text, showing checkboxes, etc.)
     * We don't care about anything other than text, so we only want Qt to execute the data() call that
     * displays text, which is done by passing it the Qt::DisplayRole role. */
    if (role == Qt::DisplayRole) {
        switch (index.column()) {
        case 0: return positionEntry.index;
        case 1: return positionEntry.alias;
        case 2: return QString::number(positionEntry.x, 'f', 2);
        case 3: return QString::number(positionEntry.y, 'f', 2);
        case 4: return QString::number(positionEntry.z, 'f', 2);
        default: return QVariant();  // QVariant() returns an invalid QVariant.
        }
    }

    return QVariant();
}

QVariant PositionTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal) {
        switch (section) {
            case 0: return "Index";
            case 1: return "Alias";
            case 2: return "X";
            case 3: return "Y";
            case 4: return "Z";
            default: return QVariant();   // Placeholder while I figure out a default return value
        }
    }

    return QVariant();

}

// REGULAR FUNCTIONS

void PositionTableModel::addRow(const SavedPosition &positionData) {
    beginInsertRows(QModelIndex(), rowCount(), rowCount());
    savedPositions.append(positionData);
    endInsertRows();
}
