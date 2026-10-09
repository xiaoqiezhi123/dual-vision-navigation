# import trimesh

# # 加载原始STL文件
# mesh = trimesh.load("/home/jc/Documents/mini_phybot_cpg/legged_gym/resources/robots/mini_phybot/meshes/left_toe.STL")
# print(f"当前面片数: {len(mesh.faces)}")

# # 直接指定目标面片数（100000）
# simplified = mesh.simplify_quadric_decimation(face_count=100000)  # 关键修改：传递面片数而非比例
# simplified.export("/home/jc/Documents/mini_phybot_cpg/legged_gym/resources/robots/mini_phybot/meshes/left_toe.STL")

# # 验证简化结果
# simplified_mesh = trimesh.load("/home/jc/Documents/mini_phybot_cpg/legged_gym/resources/robots/mini_phybot/meshes/left_toe.STL")
# print(f"简化后面片数: {len(simplified_mesh.faces)}")  # 应输出约100000

# import trimesh
# import numpy as np

# # 加载STL文件（可能返回Scene对象）
# scene_or_mesh = trimesh.load("/home/jc/Documents/mini_phybot_cpg/legged_gym/resources/robots/mini_phybot/meshes/left_toe.STL")

# # 检查加载的是Scene还是单个Mesh
# if isinstance(scene_or_mesh, trimesh.Scene):
#     print("检测到Scene对象，正在合并所有网格...")
#     # 合并场景中的所有网格
#     mesh = scene_or_mesh.dump(concatenate=True)
# else:
#     mesh = scene_or_mesh

# print(f"当前面片数: {len(mesh.faces)}")

# # 简化网格（确保面片数不超过200000）
# target_faces = min(100000, len(mesh.faces))  # 避免目标超过当前面片数
# simplified = mesh.simplify_quadric_decimation(face_count=target_faces)

# # 导出简化后的网格（注意：不要覆盖原文件！先保存到新文件）
# output_path = "/home/jc/Documents/mini_phybot_cpg/legged_gym/resources/robots/mini_phybot/meshes/left_toe_simplified.STL"
# simplified.export(output_path)

# # 验证结果
# simplified_mesh = trimesh.load(output_path)
# if isinstance(simplified_mesh, trimesh.Scene):
#     simplified_mesh = simplified_mesh.dump(concatenate=True)
# print(f"简化后面片数: {len(simplified_mesh.faces)}")