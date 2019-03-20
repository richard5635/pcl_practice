#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>

// Refer to a successful example https://www.youtube.com/watch?v=_sM5ZMJ0XGA

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCLViewer w;
  w.show ();

  return a.exec ();
}