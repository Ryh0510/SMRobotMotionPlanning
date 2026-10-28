#include <string>
#include <vector>
#include <windows.h>
#include <wininet.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <direct.h>
#include <io.h>
#include <sys/stat.h>


#ifdef POSTMODEL_SHAREDLIB_EXPORTS
#define FTPSEND_API __declspec(dllexport)
#else
#define FTPSEND_API __declspec(dllimport)
#endif

class FTPSEND_API FTPConnection {
private:
    void* hInternet;
    void* hFtpSession;
    std::string server;
    std::string username;
    std::string password;
    int port;

public:
    FTPConnection(const std::string& server, int port = 21,
        const std::string& username = "", const std::string& password = "");
    ~FTPConnection();

    bool connect();
    bool disconnect();
    bool isConnected() const;

    bool uploadFile(const std::string& localFile, const std::string& remoteFile);
    bool deleteFile(const std::string& remoteFile);
    bool createDirectory(const std::string& remoteDir);
    bool removeDirectory(const std::string& remoteDir);
    bool setCurrentDirectory(const std::string& remoteDir);

    static std::string getErrorMessage(DWORD errorCode);
};

// 文件系统辅助函数
namespace FileSystem {
    FTPSEND_API bool isFile(const std::string& path);
    FTPSEND_API bool isDirectory(const std::string& path);
    FTPSEND_API std::string getFileName(const std::string& path);
    FTPSEND_API std::string getParentPath(const std::string& path);
    FTPSEND_API bool createDirectory(const std::string& path);
    FTPSEND_API std::vector<std::string> listFiles(const std::string& path, bool recursive = false);
}

/**
 * @brief 删除FTP服务器上的文件
 */
FTPSEND_API bool RemoveFileFTP(FTPConnection& ftp, const std::string& filepath);

/**
 * @brief 递归删除FTP服务器上的目录
 */
FTPSEND_API bool RemoveDirFTP(FTPConnection& ftp, const std::string& path);

/**
 * @brief 上传文件到FTP服务器
 */
FTPSEND_API bool UploadFileFTP(FTPConnection& ftp, const std::string& local_file_path, const std::string& remote_path);

/**
 * @brief 上传目录到FTP服务器（递归）
 */
FTPSEND_API bool UploadDirFTP(FTPConnection& ftp, const std::string& local_path, const std::string& remote_path);

/**
 * @brief 上传文件或目录到FTP服务器
 */
FTPSEND_API bool UploadFTP(FTPConnection& ftp, const std::string& path, const std::string& remote_path, int pause_sec = 2);

/**
 * @brief 上传多个文件或目录到FTP服务器
 */
FTPSEND_API bool UploadFTP(FTPConnection& ftp, const std::vector<std::string>& paths, const std::string& remote_path, int pause_sec = 2);

/**
 * @brief 完整的FTP上传函数（包含连接管理）
 */
FTPSEND_API bool UploadFTP(const std::string& path,
    const std::string& server_ip,
    const std::string& remote_path,
    const std::string& username,
    const std::string& password,
    int pause_sec = 2);

/**
 * @brief 完整的FTP上传函数（多个文件，包含连接管理）
 */
FTPSEND_API bool UploadFTP(const std::vector<std::string>& paths,
    const std::string& server_ip,
    const std::string& remote_path,
    const std::string& username,
    const std::string& password,
    int pause_sec = 2);