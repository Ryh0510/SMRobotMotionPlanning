#include <PostModel/ftpsend.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <direct.h>
#include <io.h>
#include <sys/stat.h>

using namespace std::chrono_literals;
#pragma comment(lib, "wininet.lib")

// 文件系统辅助函数实现
namespace FileSystem {

    bool isFile(const std::string& path) {
        struct _stat fileStat;
        if (_stat(path.c_str(), &fileStat) != 0) {
            return false;
        }
        return (fileStat.st_mode & _S_IFREG) != 0;
    }

    bool isDirectory(const std::string& path) {
        struct _stat fileStat;
        if (_stat(path.c_str(), &fileStat) != 0) {
            return false;
        }
        return (fileStat.st_mode & _S_IFDIR) != 0;
    }

    std::string getFileName(const std::string& path) {
        size_t pos = path.find_last_of("\\/");
        if (pos == std::string::npos) {
            return path;
        }
        return path.substr(pos + 1);
    }

    std::string getParentPath(const std::string& path) {
        size_t pos = path.find_last_of("\\/");
        if (pos == std::string::npos) {
            return "";
        }
        return path.substr(0, pos);
    }

    bool createDirectory(const std::string& path) {
        return _mkdir(path.c_str()) == 0;
    }

    std::vector<std::string> listFiles(const std::string& path, bool recursive) {
        std::vector<std::string> files;

        std::string searchPath = path + "\\*";
        _finddata_t fileInfo;
        intptr_t handle = _findfirst(searchPath.c_str(), &fileInfo);

        if (handle == -1) {
            return files;
        }

        do {
            if (strcmp(fileInfo.name, ".") == 0 || strcmp(fileInfo.name, "..") == 0) {
                continue;
            }

            std::string fullPath = path + "\\" + fileInfo.name;

            if ((fileInfo.attrib & _A_SUBDIR) != 0) {
                // 如果是目录且需要递归
                if (recursive) {
                    auto subFiles = listFiles(fullPath, true);
                    files.insert(files.end(), subFiles.begin(), subFiles.end());
                }
            }
            else {
                files.push_back(fullPath);
            }
        } while (_findnext(handle, &fileInfo) == 0);

        _findclose(handle);
        return files;
    }

} // namespace FileSystem

// FTPConnection 类实现
FTPConnection::FTPConnection(const std::string& server, int port,
    const std::string& username, const std::string& password)
    : server(server), port(port), username(username), password(password),
    hInternet(nullptr), hFtpSession(nullptr) {
}

FTPConnection::~FTPConnection() {
    disconnect();
}

bool FTPConnection::connect() {
    // 初始化WinINet
    hInternet = InternetOpenA("FTPUpload", INTERNET_OPEN_TYPE_DIRECT, nullptr, nullptr, 0);
    if (!hInternet) {
        std::cerr << "InternetOpen failed: " << GetLastError() << std::endl;
        return false;
    }

    // 连接到FTP服务器
    hFtpSession = InternetConnectA(hInternet, server.c_str(), port,
        username.empty() ? nullptr : username.c_str(),
        password.empty() ? nullptr : password.c_str(),
        INTERNET_SERVICE_FTP, INTERNET_FLAG_PASSIVE, 0);
    if (!hFtpSession) {
        std::cerr << "InternetConnect failed: " << GetLastError() << std::endl;
        InternetCloseHandle(hInternet);
        hInternet = nullptr;
        return false;
    }

    return true;
}

bool FTPConnection::disconnect() {
    if (hFtpSession) {
        InternetCloseHandle(hFtpSession);
        hFtpSession = nullptr;
    }
    if (hInternet) {
        InternetCloseHandle(hInternet);
        hInternet = nullptr;
    }
    return true;
}

bool FTPConnection::isConnected() const {
    return hFtpSession != nullptr;
}

bool FTPConnection::uploadFile(const std::string& localFile, const std::string& remoteFile) {
    if (!hFtpSession) return false;

    // 上传文件
    if (!FtpPutFileA(hFtpSession, localFile.c_str(), remoteFile.c_str(),
        FTP_TRANSFER_TYPE_BINARY, 0)) {
        std::cerr << "FtpPutFile failed: " << GetLastError() << std::endl;
        return false;
    }

    return true;
}

bool FTPConnection::createDirectory(const std::string& remoteDir) {
    if (!hFtpSession) return false;

    if (!FtpCreateDirectoryA(hFtpSession, remoteDir.c_str())) {
        std::cerr << "FtpCreateDirectory failed: " << GetLastError() << std::endl;
        return false;
    }

    return true;
}

bool FTPConnection::setCurrentDirectory(const std::string& remoteDir) {
    if (!hFtpSession) return false;

    if (!FtpSetCurrentDirectoryA(hFtpSession, remoteDir.c_str())) {
        std::cerr << "FtpSetCurrentDirectory failed: " << GetLastError() << std::endl;
        return false;
    }

    return true;
}

bool FTPConnection::deleteFile(const std::string& remoteFile) {
    if (!hFtpSession) return false;

    if (!FtpDeleteFileA(hFtpSession, remoteFile.c_str())) {
        std::cerr << "FtpDeleteFile failed: " << GetLastError() << std::endl;
        return false;
    }

    return true;
}

bool FTPConnection::removeDirectory(const std::string& remoteDir) {
    if (!hFtpSession) return false;

    if (!FtpRemoveDirectoryA(hFtpSession, remoteDir.c_str())) {
        std::cerr << "FtpRemoveDirectory failed: " << GetLastError() << std::endl;
        return false;
    }

    return true;
}

std::string FTPConnection::getErrorMessage(DWORD errorCode) {
    char buffer[256];
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, nullptr, errorCode, 0, buffer, sizeof(buffer), nullptr);
    return std::string(buffer);
}

// 辅助函数实现
bool RemoveFileFTP(FTPConnection& ftp, const std::string& filepath) {
    return ftp.deleteFile(filepath);
}

bool RemoveDirFTP(FTPConnection& ftp, const std::string& path) {
    return ftp.removeDirectory(path);
}

bool UploadFileFTP(FTPConnection& ftp, const std::string& local_file_path, const std::string& remote_path) {
    std::string filename = FileSystem::getFileName(local_file_path);
    std::string remote_file = remote_path + "/" + filename;
    return ftp.uploadFile(local_file_path, remote_file);
}

bool UploadDirFTP(FTPConnection& ftp, const std::string& local_path, const std::string& remote_path) {
    // 创建远程目录
    std::string dirname = FileSystem::getFileName(local_path);
    std::string remote_dir = remote_path + "/" + dirname;

    if (!ftp.createDirectory(remote_dir)) {
        return false;
    }

    // 递归上传文件
    auto files = FileSystem::listFiles(local_path, true);
    for (const auto& local_file : files) {
        // 计算相对路径
        std::string relative_path = local_file.substr(local_path.length() + 1);
        std::string remote_file_path = remote_dir + "/" + relative_path;

        // 创建子目录（如果需要）
        std::string remote_subdir = FileSystem::getParentPath(remote_file_path);
        if (!remote_subdir.empty() && !ftp.setCurrentDirectory(remote_subdir)) {
            if (!ftp.createDirectory(remote_subdir)) {
                std::cerr << "Failed to create remote directory: " << remote_subdir << std::endl;
                return false;
            }
        }

        if (!ftp.uploadFile(local_file, remote_file_path)) {
            std::cerr << "Failed to upload file: " << local_file << std::endl;
            return false;
        }
    }

    return true;
}

bool UploadFTP(FTPConnection& ftp, const std::string& path, const std::string& remote_path, int pause_sec) {
    if (FileSystem::isFile(path)) {
        std::cout << "Sending program file " << path << "..." << std::endl;
        return UploadFileFTP(ftp, path, remote_path);
    }
    else if (FileSystem::isDirectory(path)) {
        std::cout << "Sending program folder " << path << "..." << std::endl;
        return UploadDirFTP(ftp, path, remote_path);
    }
    else {
        std::cerr << "Path is neither a file nor a directory: " << path << std::endl;
        return false;
    }

    if (pause_sec > 0) {
        std::this_thread::sleep_for(std::chrono::seconds(pause_sec));
    }

    return true;
}

bool UploadFTP(FTPConnection& ftp, const std::vector<std::string>& paths, const std::string& remote_path, int pause_sec) {
    if (paths.empty()) {
        std::cout << "Nothing to transfer" << std::endl;
        return true;
    }

    bool all_success = true;
    for (const auto& path : paths) {
        if (!UploadFTP(ftp, path, remote_path, 0)) {
            all_success = false;
        }
    }

    if (all_success) {
        std::cout << paths.size() << " files and folders successfully transferred" << std::endl;
    }

    if (pause_sec > 0) {
        std::this_thread::sleep_for(std::chrono::seconds(pause_sec));
    }

    return all_success;
}

bool UploadFTP(const std::string& path,
    const std::string& server_ip,
    const std::string& remote_path,
    const std::string& username,
    const std::string& password,
    int pause_sec) {

    FTPConnection ftp(server_ip, 21, username, password);
    if (!ftp.connect()) {
        return false;
    }

    bool result = UploadFTP(ftp, path, remote_path, pause_sec);
    ftp.disconnect();
    return result;
}

bool UploadFTP(const std::vector<std::string>& paths,
    const std::string& server_ip,
    const std::string& remote_path,
    const std::string& username,
    const std::string& password,
    int pause_sec) {

    FTPConnection ftp(server_ip, 21, username, password);
    if (!ftp.connect()) {
        return false;
    }

    bool result = UploadFTP(ftp, paths, remote_path, pause_sec);
    ftp.disconnect();
    return result;
}
