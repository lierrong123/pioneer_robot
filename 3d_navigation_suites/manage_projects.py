#!/usr/bin/env python3
"""
3D导航套件项目管理脚本
"""

import os
import subprocess
import sys
from pathlib import Path

class ProjectManager:
    def __init__(self):
        self.ws_root = Path.home() / "lierrong_ws"
        self.src_dir = self.ws_root / "src"
        self.suites_dir = self.src_dir / "3d_navigation_suites"
        
        self.projects = {
            "A": {
                "name": "scheme_a_2d_projection",
                "desc": "八叉树投影为2D导航",
                "status": "ready"
            },
            "B": {
                "name": "scheme_b_realtime_fusion",
                "desc": "实时八叉树融合导航",
                "status": "framework"
            },
            "C": {
                "name": "scheme_c_vox_nav",
                "desc": "VoxNav 3D导航",
                "status": "framework"
            }
        }
    
    def show_menu(self):
        print("="*60)
        print("          3D导航套件项目管理")
        print("="*60)
        print()
        
        for key, project in self.projects.items():
            status_icon = "✅" if project["status"] == "ready" else "🔄"
            print(f"  {key}. {status_icon} {project['name']}")
            print(f"     描述: {project['desc']}")
            print()
        
        print("="*60)
        print("  1. 构建所有项目")
        print("  2. 构建单个项目")
        print("  3. 清理构建文件")
        print("  4. 运行项目")
        print("  5. 查看项目状态")
        print("  0. 退出")
        print("="*60)
    
    def run_command(self, cmd, cwd=None):
        """运行命令并显示输出"""
        try:
            result = subprocess.run(
                cmd, 
                shell=True, 
                cwd=cwd,
                capture_output=True, 
                text=True
            )
            print(result.stdout)
            if result.stderr:
                print("错误:", result.stderr)
            return result.returncode == 0
        except Exception as e:
            print(f"执行命令失败: {e}")
            return False
    
    def build_all(self):
        """构建所有项目"""
        print("正在构建所有3D导航项目...")
        
        # 切换到工作空间目录
        os.chdir(self.ws_root)
        
        # 构建所有包
        cmd = "colcon build --symlink-install"
        if self.run_command(cmd):
            print("✅ 所有项目构建成功！")
        else:
            print("❌ 构建失败")
    
    def build_single(self, project_name):
        """构建单个项目"""
        if project_name not in [p["name"] for p in self.projects.values()]:
            print(f"❌ 项目 {project_name} 不存在")
            return
        
        print(f"正在构建 {project_name}...")
        os.chdir(self.ws_root)
        
        cmd = f"colcon build --packages-select {project_name} --symlink-install"
        if self.run_command(cmd):
            print(f"✅ {project_name} 构建成功！")
        else:
            print(f"❌ {project_name} 构建失败")
    
    def clean_build(self):
        """清理构建文件"""
        print("清理构建文件...")
        os.chdir(self.ws_root)
        
        cmd = "rm -rf build install log"
        if self.run_command(cmd):
            print("✅ 清理完成")
        else:
            print("❌ 清理失败")
    
    def run_project(self, project_name):
        """运行项目"""
        if project_name == "scheme_a_2d_projection":
            print("运行方案A：")
            print("  ros2 launch scheme_a_2d_projection navigation.launch.py")
            print()
            print("可选参数：")
            print("  octomap_file:=/path/to/map.bt")
            print("  use_sim_time:=false (真实机器人)")
        elif project_name == "scheme_b_realtime_fusion":
            print("方案B：实时融合导航（待实现）")
        elif project_name == "scheme_c_vox_nav":
            print("方案C：VoxNav 3D导航（待实现）")
        else:
            print(f"未知项目: {project_name}")
    
    def show_status(self):
        """显示项目状态"""
        print("项目状态检查：")
        print("-"*40)
        
        for key, project in self.projects.items():
            project_path = self.suites_dir / project["name"]
            if project_path.exists():
                # 检查package.xml
                pkg_xml = project_path / "package.xml"
                if pkg_xml.exists():
                    status = "✅ 配置完整"
                else:
                    status = "⚠️  package.xml缺失"
            else:
                status = "❌ 目录不存在"
            
            print(f"{project['name']}: {status}")
        
        print("-"*40)
    
    def main(self):
        """主循环"""
        while True:
            self.show_menu()
            choice = input("请选择操作: ").strip()
            
            if choice == "0":
                print("退出管理工具")
                break
            elif choice == "1":
                self.build_all()
            elif choice == "2":
                project_key = input("选择项目 (A/B/C): ").strip().upper()
                if project_key in self.projects:
                    self.build_single(self.projects[project_key]["name"])
                else:
                    print("无效选择")
            elif choice == "3":
                self.clean_build()
            elif choice == "4":
                project_key = input("选择项目 (A/B/C): ").strip().upper()
                if project_key in self.projects:
                    self.run_project(self.projects[project_key]["name"])
                else:
                    print("无效选择")
            elif choice == "5":
                self.show_status()
            else:
                print("无效选择，请重试")
            
            input("\n按Enter键继续...")

if __name__ == "__main__":
    manager = ProjectManager()
    manager.main()