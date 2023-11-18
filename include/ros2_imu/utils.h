/**
 * @file utils.h
 * @author 黄李全 (846863428@qq.com)
 * @brief 封装一些小工具
 * @version 0.1
 * @date 2023-01-12
 * @copyright Copyright (c) {2023} 个人版权所有,仅供学习
 */
#ifndef __UTILS_H__
#define __UTILS_H__

#include <iostream>
#include <vector>

namespace Utils {
 /**
 * @brief 搜索设备 iio 设备
 * @param name 模块名称
 * @return std::string 返回路径
 */
std::string ScanIioDevice(std::string name);

/**
 * @brief 一次性读取文件所有内容
 * @param path 文件路径
 * @return std::string 内容
 */
std::string ReadFileIntoString(const std::string& path);

/**
 * @brief 获取文件夹下所有文件
 * @param path 
 * @param files 
 */
void getFiles(std::string path, std::vector<std::string>& files);

/**
 * @brief 字节转文本
 * @param data
 * @param len
 * @return std::string
 */
std::string Bytes2String(uint8_t *data, uint32_t len);

/**
 * @brief 工具函数，用于计算循环冗余校验码
 * @param pContent 指向需要计算校验码的数据
 * @param usLength 数据的字节长度
 * @return uint16_t 
 */
uint16_t CheckCRC(uint8_t *pContent, uint16_t usLength);
}

#endif
