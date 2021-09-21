
#include "mainapplication.hpp"
#include <QApplication>
#include <QCommandLineParser>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QGuiApplication>
#include <QLoggingCategory>
#include <QQmlApplicationEngine>
#include <QScopedPointer>
#include <QTextStream>
#include <signal.h>
// Smart pointer to log file
QScopedPointer<QFile> m_logFile;
/**
 * @brief unixSignalHandler
 * @param signum
 * @author Louis P Meadows
 * @date September 15th 2021
 * @copyright Sciton, Inc
 */
static void unixSignalHandler(int signum) {
  qDebug("[QML] main.cpp::unixSignalHandler(). signal = %s", strsignal(signum));

  /*
   * Make sure sciton Boson application gracefully quits.
   * NOTE - purpose for calling qApp->exit(0):
   *      1. Forces the Qt framework's "main event loop `qApp->exec()`" to quit
   * looping.
   *      2. Also emits the QGuiApplication::aboutToQuit() signal. This signal
   * is used for cleanup code.
   */
  qApp->exit(0);
}
/**
 * @brief messageLogger
 * @param type
 * @param context
 * @param msg
 * @author Louis P Meadows
 * @date September 15th 2021
 * @copyright Sciton, Inc
 */
void messageLogger(QtMsgType type, const QMessageLogContext &context,
                   const QString &msg) {
  QByteArray localMsg = msg.toLocal8Bit();
  const char *file = context.file ? context.file : "";
  const char *function = context.function ? context.function : "";
  // Open stream file writes
  QTextStream out(m_logFile.data());
  // Write the date of recording
  out << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz ");
  // By type determine to what level belongs message
  switch (type) {
  case QtDebugMsg:
    fprintf(stderr, "Debug: %s (%s:%u, %s)\n", localMsg.constData(), file,
            context.line, function);
    out << "Debug: ";
    break;

  case QtWarningMsg:
    fprintf(stderr, "Warning: %s (%s:%u, %s)\n", localMsg.constData(), file,
            context.line, function);
    out << "Warning: ";
    break;
  case QtCriticalMsg:
    fprintf(stderr, "Critical: %s (%s:%u, %s)\n", localMsg.constData(), file,
            context.line, function);
    out << "Critical: ";
    break;
  case QtFatalMsg:
    fprintf(stderr, "Fatal: %s (%s:%u, %s)\n", localMsg.constData(), file,
            context.line, function);
    out << "Fatal: ";
    break;
  default:
    fprintf(stderr, "Info: %s (%s:%u, %s)\n", localMsg.constData(), file,
            context.line, function);
    out << "Info: ";
    break;
  }
  // Write to the output of the message
  out << msg << endl;
  out.flush(); // Clear the buffered data
}

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 * @author Louis P Meadows
 * @date September 15th 2021
 * @copyright Sciton, Inc
 */
int main(int argc, char *argv[]) {
  // Set the logging file
  // check which a path to file you use
  m_logFile.reset(new QFile("logFile.txt"));
  // Open the file logging
  m_logFile.data()->open(QFile::WriteOnly | QFile::Text);
  // Set handler
  qInstallMessageHandler(messageLogger);
  QCommandLineParser parser;
  parser.setApplicationDescription(
      "Sciton Boson, USB interface to Flir camera");
  parser.addHelpOption();
  parser.addVersionOption();
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

  QApplication app(argc, argv);
  QGuiApplication::setOverrideCursor(QCursor(Qt::BlankCursor));
  QGuiApplication::setOrganizationName("Sciton Inc");
  QGuiApplication::setOrganizationDomain("sciton.com");
  QGuiApplication::setApplicationName("Sciton Boson");

  MainApplication *mainApplication = new MainApplication();
  qDebug() << __FUNCTION__ << __LINE__ << "Main Application Created ";
  mainApplication->print_help();
  QQmlApplicationEngine engine;

  const QUrl url(QStringLiteral("qrc:/qml/main.qml"));
  QObject::connect(&engine, &QQmlApplicationEngine::objectCreated, &app,
                   [url](QObject *obj, const QUrl &objUrl) {
                     if (!obj && url == objUrl)
                       QCoreApplication::exit(-1);
                   },
                   Qt::QueuedConnection);
  engine.load(url);
  /* Set a signal handler for a power down or a control-c */
  if (signal(SIGTERM, unixSignalHandler) == SIG_ERR) {
    qDebug()
        << "[QML] an error occurred while setting a signal terminate handler";
  }
  if (signal(SIGINT, unixSignalHandler) == SIG_ERR) {
    qDebug()
        << "[QML] an error occurred while setting a signal interrupt handler";
  }
  QObject::connect(&app, SIGNAL(aboutToQuit()), mainApplication,
                   SLOT(handleSigTerm()));
  return app.exec();
}
