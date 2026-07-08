/* Copyright (c) 2026, Adrian Lopez <adrianlopezroche@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */
#pragma once
#include <QDialog>

namespace Ui {
class PaperTrackerHelpDialog;
}

class PaperTrackerHelpDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PaperTrackerHelpDialog(QWidget *parent = nullptr);
    ~PaperTrackerHelpDialog();

private:
    Ui::PaperTrackerHelpDialog *ui;
    void loadHelpText(const QString &filePath);
    QString removeSpaces(QString &input);
};