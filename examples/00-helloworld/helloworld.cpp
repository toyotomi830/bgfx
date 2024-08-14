/*
 * Copyright 2011-2024 Branimir Karadzic. All rights reserved.
 * License: https://github.com/bkaradzic/bgfx/blob/master/LICENSE
 */
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif // !GLM_ENABLE_EXPERIMENTAL

#include "common.h"
#include "bgfx_utils.h"
#include <entry/cmd.h>
#include <entry/input.h>
#include <debugdraw/debugdraw.h>
#include "camera.h"
#include "imgui/imgui.h"
#include <bx/uint32_t.h>

#include <vector>
#include <unordered_map>
#include "SceneMgr.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <bx/math.h>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <string>
#include <memory>
#include <glm/gtc/type_ptr.hpp>

#include "SphereSegmentation.h"
#include "PlaneSegmentation.h"
#include "CylinderSegmentation.h"
#include"Voxelization.h"
#include "SpanData.h"




namespace
{
	/*
	* 得到一个随机的颜色
	* 返回值：uint32_t
	*/
	uint32_t generateRandomColor()
	{
		uint8_t red = std::rand() % 127;
		uint8_t green = std::rand() % 127;
		uint8_t blue = std::rand() % 127;
		return blue | (green << 8) | (red << 16) | 0xff000000;
	}
	/*
	* 使用顶点着色的顶点结构
	*/
	struct PosColorVertex
	{
		float m_x;
		float m_y;
		float m_z;
		uint32_t m_abgr;

		static void init()
		{
			ms_layout
				.begin()
				.add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
				.add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
				.end();
		};
		static bgfx::VertexLayout ms_layout;
	};

	bgfx::VertexLayout PosColorVertex::ms_layout;
	/*
	* 体素Box的顶点
	*/
	static PosColorVertex s_cubeVertices[] =
	{
		{-1.0f,  1.0f,  1.0f, 0xff000000 },
		{ 1.0f,  1.0f,  1.0f, 0xff0000ff },
		{-1.0f, -1.0f,  1.0f, 0xff00ff00 },
		{ 1.0f, -1.0f,  1.0f, 0xff00ffff },
		{-1.0f,  1.0f, -1.0f, 0xffff0000 },
		{ 1.0f,  1.0f, -1.0f, 0xffff00ff },
		{-1.0f, -1.0f, -1.0f, 0xffffff00 },
		{ 1.0f, -1.0f, -1.0f, 0xffffffff },
	};
	/*
	* 体素Box的Indices
	*/
	static const uint16_t s_cubeTriList[] =
	{
		0, 1, 2, // 0
		1, 3, 2,
		4, 6, 5, // 2
		5, 6, 7,
		0, 2, 4, // 4
		4, 2, 6,
		1, 5, 3, // 6
		5, 7, 3,
		0, 4, 1, // 8
		4, 5, 1,
		2, 3, 6, // 10
		6, 3, 7,
	};

	/*
	* Mesh的数据结构
	* vertices和indices初始化后就没用了
	*/
	struct RenderMesh
	{
		std::vector<PosColorVertex> vertices;
		std::vector<uint16_t> indices;
		bgfx::VertexBufferHandle vertexBuffer;
		bgfx::IndexBufferHandle indicesBuffer;
		RenderMesh(const std::vector<bx::Vec3>& vers, const std::vector< unsigned int>& in)
		{
			vertices.reserve(vers.size());
			indices.reserve(indices.size());
			for (auto&& i : vers)
			{
				vertices.emplace_back(PosColorVertex({ i.x,i.y,i.z,generateRandomColor() }));
			}
			for (auto&& j : in)
			{
				indices.emplace_back(j);
			}
			vertexBuffer = bgfx::createVertexBuffer(
				// Static data can be passed with bgfx::makeRef
				bgfx::copy(vertices.data(), sizeof(PosColorVertex) * vertices.size())
				, PosColorVertex::ms_layout
			);

			// Create static index buffer for triangle list rendering.
			indicesBuffer = bgfx::createIndexBuffer(
				// Static data can be passed with bgfx::makeRef
				bgfx::copy(indices.data(), sizeof(uint16_t) * indices.size())
			);
			std::vector<PosColorVertex>().swap(vertices);
			std::vector<uint16_t>().swap(indices);

		}
		RenderMesh(const std::vector<glm::vec3>& vers, const std::vector<int>& in)
		{
			vertices.reserve(vers.size());
			indices.reserve(indices.size());
			for (auto&& i : vers)
			{
				vertices.emplace_back(PosColorVertex({ i.x,i.y,i.z,generateRandomColor() }));
			}
			for (auto&& j : in)
			{
				indices.emplace_back(j);
			}
			vertexBuffer = bgfx::createVertexBuffer(
				// Static data can be passed with bgfx::makeRef
				bgfx::copy(vertices.data(), sizeof(PosColorVertex) * vertices.size())
				, PosColorVertex::ms_layout
			);

			// Create static index buffer for triangle list rendering.
			indicesBuffer = bgfx::createIndexBuffer(
				// Static data can be passed with bgfx::makeRef
				bgfx::copy(indices.data(), sizeof(uint16_t) * indices.size())
			);
			std::vector<PosColorVertex>().swap(vertices);
			std::vector<uint16_t>().swap(indices);

		}
	};



	class ExampleCubes : public entry::AppI
	{
	public:
		ExampleCubes(const char* _name, const char* _description, const char* _url)
			: entry::AppI(_name, _description, _url)
		{

		}

		void init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height) override
		{
			Args args(_argc, _argv);

			m_width = _width;
			m_height = _height;
			m_debug = BGFX_DEBUG_NONE;
			m_reset = BGFX_RESET_VSYNC;

			bgfx::Init init;
			init.type = args.m_type;
			init.vendorId = args.m_pciId;
			init.platformData.nwh = entry::getNativeWindowHandle(entry::kDefaultWindowHandle);
			init.platformData.ndt = entry::getNativeDisplayHandle();
			init.platformData.type = entry::getNativeWindowHandleType();
			init.resolution.width = m_width;
			init.resolution.height = m_height;
			init.resolution.reset = m_reset;
			bgfx::init(init);

			// Enable debug text.
			bgfx::setDebug(m_debug);

			bgfx::setViewClear(0
				, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH
				, 0x303030ff
				, 1.0f
				, 0
			);

			ddInit();
			cameraCreate();
			cameraSetPosition({ 2399.9,-1114.7,1597 });
			cameraSetVerticalAngle(0.0f);

			// Create vertex stream declaration.
			PosColorVertex::init();

			CubeVertexBuffer = bgfx::createVertexBuffer(
				// Static data can be passed with bgfx::makeRef
				bgfx::makeRef(s_cubeVertices, sizeof(s_cubeVertices))
				, PosColorVertex::ms_layout
			);

			// Create static index buffer for triangle list rendering.
			CubeIndicesBuffer = bgfx::createIndexBuffer(
				// Static data can be passed with bgfx::makeRef
				bgfx::makeRef(s_cubeTriList, sizeof(s_cubeTriList))
			);
			SceneMgrPtr = std::make_unique<SceneMgr>();
			SceneMgrPtr->LoadJsonData();
			auto&& Geos = SceneMgrPtr->MeshMap;
			for (auto it = Geos.begin(); it != Geos.end(); it++)
			{
				Meshes[it->first] = std::make_shared<RenderMesh>(it->second->worldVertices, it->second->indices);
			}
			Spheres.emplace_back(std::make_shared<segment_mgr::SphereSegmentMgr>(1500.0f,2,32.0f));
			//Spheres.emplace_back(std::make_shared<segment_mgr::PlaneSegmentMgr>(3000, 4, 16.0f));
			//Spheres.emplace_back(std::make_shared<segment_mgr::CylinderSegmentMgr>(1500.0f, 4000, 1, 32.0f));

			voxelFuncs::ReCastSphereVoxelization(SceneMgrPtr, Spheres[0], Spheres[0]->GetTileSize(), Spheres[0]->GetCellSize(), Spheres[0]->GetCellSize(), -500, 1000.0f);


			// Create program from shaders.
			m_program = loadProgram("vs_cubes", "fs_cubes");
			m_program_instanced = loadProgram("vs_instancing", "fs_instancing");
			m_timeOffset = bx::getHPCounter();

			imguiCreate();
		}

		virtual int shutdown() override
		{
			imguiDestroy();
			cameraDestroy();
			// Cleanup.

			for (auto it = Meshes.begin(); it != Meshes.end(); it++)
			{
				bgfx::destroy(it->second->vertexBuffer);
				bgfx::destroy(it->second->indicesBuffer);
			}
			bgfx::destroy(m_program);

			// Shutdown bgfx.
			bgfx::shutdown();

			return 0;
		}

		bool update() override
		{
			if (!entry::processEvents(m_width, m_height, m_debug, m_reset, &m_mouseState))
			{
				imguiBeginFrame(m_mouseState.m_mx
					, m_mouseState.m_my
					, (m_mouseState.m_buttons[entry::MouseButton::Left] ? IMGUI_MBUT_LEFT : 0)
					| (m_mouseState.m_buttons[entry::MouseButton::Right] ? IMGUI_MBUT_RIGHT : 0)
					| (m_mouseState.m_buttons[entry::MouseButton::Middle] ? IMGUI_MBUT_MIDDLE : 0)
					, m_mouseState.m_mz
					, uint16_t(m_width)
					, uint16_t(m_height)
				);
				showExampleDialog(this);

				ImGui::SetNextWindowPos(
					ImVec2(m_width - m_width / 5.0f, 10.0f)
					, ImGuiCond_FirstUseEver
				);
				ImGui::SetNextWindowSize(
					ImVec2(m_width / 3.0f, m_height / 1.5f)
					, ImGuiCond_FirstUseEver
				);
				ImGui::Begin("Settings"
					, NULL
					, 0
				);

				ImGui::Text("Primitive topology:");

				ImGui::InputFloat3("Camera Pos ", cameraPosNow);
				cameraPosNow[0] = cameraGetPosition().x;
				cameraPosNow[1] = cameraGetPosition().y;
				cameraPosNow[2] = cameraGetPosition().z;
				
				Span f;
				Span t;
				if (ImGui::Button("Gen Random Point"))
				{
					
					way.swap(std::vector<std::shared_ptr<voxelFuncs::wayNode>>());
					while (way.empty())
					{
						
						f = voxelFuncs::getRandomSpan(*Spheres[0]);
						t = voxelFuncs::getRandomSpan(*Spheres[0]);
						way = voxelFuncs::findWays(f, t, *Spheres[0]);
						if (way.empty())
							continue;

						const segment_mgr::Tile& fromTile = Spheres[0]->GetTileByIndex(SpanData::getInstance().Data[f.ListIndex].TileIndex);
						const segment_mgr::Tile& toTile = Spheres[0]->GetTileByIndex(SpanData::getInstance().Data[t.ListIndex].TileIndex);
						glm::vec3 axis_y1 = glm::normalize(glm::cross(fromTile.axis_u, fromTile.axis_v));
						glm::vec3 axis_y2 = glm::normalize(glm::cross(toTile.axis_u, toTile.axis_v));

						fromPos = SpanData::getInstance().Data[f.ListIndex].CenteralWorldPos + (axis_y1 * f.top);
						toPos = SpanData::getInstance().Data[t.ListIndex].CenteralWorldPos + (axis_y2 * t.top);
					}
					
					
					way.swap(std::vector<std::shared_ptr<voxelFuncs::wayNode>>());
					f = voxelFuncs::getRandomSpan(*Spheres[0]);
					t = voxelFuncs::getRandomSpan(*Spheres[0]);
					way = voxelFuncs::findWays(f, t, *Spheres[0]);

					const segment_mgr::Tile& fromTile = Spheres[0]->GetTileByIndex(SpanData::getInstance().Data[f.ListIndex].TileIndex);
					const segment_mgr::Tile& toTile = Spheres[0]->GetTileByIndex(SpanData::getInstance().Data[t.ListIndex].TileIndex);
					glm::vec3 axis_y1 = glm::normalize(glm::cross(fromTile.axis_u, fromTile.axis_v));
					glm::vec3 axis_y2 = glm::normalize(glm::cross(toTile.axis_u, toTile.axis_v));

					fromPos = SpanData::getInstance().Data[f.ListIndex].CenteralWorldPos + (axis_y1 * f.top);
					toPos = SpanData::getInstance().Data[t.ListIndex].CenteralWorldPos + (axis_y2 * t.top);									
				}
				
				
				
				if (ImGui::Button("Reverse"))
				{

					way.swap(std::vector<std::shared_ptr<voxelFuncs::wayNode>>());
					way = voxelFuncs::findWays(t, f, *Spheres[0]);

					const segment_mgr::Tile& fromTile = Spheres[0]->GetTileByIndex(SpanData::getInstance().Data[t.ListIndex].TileIndex);
					const segment_mgr::Tile& toTile = Spheres[0]->GetTileByIndex(SpanData::getInstance().Data[f.ListIndex].TileIndex);
					glm::vec3 axis_y1 = glm::normalize(glm::cross(fromTile.axis_u, fromTile.axis_v));
					glm::vec3 axis_y2 = glm::normalize(glm::cross(toTile.axis_u, toTile.axis_v));

					fromPos = SpanData::getInstance().Data[t.ListIndex].CenteralWorldPos + (axis_y1 * t.top);
					toPos = SpanData::getInstance().Data[f.ListIndex].CenteralWorldPos + (axis_y2 * f.top);
				}
				

				ImGui::End();
				imguiEndFrame();


				int64_t now = bx::getHPCounter() - m_timeOffset;
				static int64_t last = now;
				const int64_t frameTime = now - last;
				last = now;
				const double freq = double(bx::getHPFrequency());
				const float deltaTime = float(frameTime / freq);

				// Update camera.
				cameraUpdate(deltaTime, m_mouseState, ImGui::MouseOverArea());

				float time = (float)((bx::getHPCounter() - m_timeOffset) / double(bx::getHPFrequency()));

				// Set view and projection matrix for view 0.

				float view[16];
				cameraGetViewMtx(view);
				float proj[16];
				bx::mtxProj(proj, 60.0f, float(m_width) / float(m_height), 10.0f, 10000, bgfx::getCaps()->homogeneousDepth);

				bgfx::setViewRect(0, 0, 0, uint16_t(m_width), uint16_t(m_height));
				bgfx::setViewTransform(0, view, proj);

				// This dummy draw call is here to make sure that view 0 is cleared
				// if no other draw calls are submitted to view 0.
				bgfx::touch(0);

				uint64_t state = 0
					| BGFX_STATE_WRITE_RGB
					| BGFX_STATE_WRITE_A
					| BGFX_STATE_WRITE_Z
					| BGFX_STATE_DEPTH_TEST_LESS
					| BGFX_STATE_CULL_CW
					| BGFX_STATE_MSAA
					;
				for (int i = 0; i < SceneMgrPtr->MeshObjects.size(); i++)
				{
					auto&& object = SceneMgrPtr->MeshObjects[i];
					glm::mat4 m;
					m = glm::translate(glm::mat4(1.0f), object.WorldPos) * glm::toMat4(object.rotation) * glm::scale(glm::mat4(1.0f), object.scale);
					bgfx::setTransform(glm::value_ptr(m));
					auto&& M = Meshes[object.MeshPathName];
					bgfx::setVertexBuffer(0, M->vertexBuffer);
					bgfx::setIndexBuffer(M->indicesBuffer);
					bgfx::setState(state);
					bgfx::submit(0, m_program);
					bgfx::setTransform(nullptr);
				}
				DrawAllVoxel(state);
				//DrawAllVoxelCylinder(state);
				//DrawAllVoxelSquere(state);
				DrawWays(state);
				glm::mat4 m;
				m = glm::translate(glm::mat4(1.0f), fromPos) * glm::scale(glm::mat4(1.0),glm::vec3(Spheres[0]->GetCellSize()));
				bgfx::setTransform(glm::value_ptr(m));
				bgfx::setVertexBuffer(0, CubeVertexBuffer);
				bgfx::setIndexBuffer(CubeIndicesBuffer);
				bgfx::setState(state);
				bgfx::submit(0, m_program);
				bgfx::setTransform(nullptr);

				m = glm::translate(glm::mat4(1.0f), toPos) * glm::scale(glm::mat4(1.0), glm::vec3(Spheres[0]->GetCellSize()));;
				bgfx::setTransform(glm::value_ptr(m));
				bgfx::setVertexBuffer(0, CubeVertexBuffer);
				bgfx::setIndexBuffer(CubeIndicesBuffer);
				bgfx::setState(state);
				bgfx::submit(0, m_program);
				bgfx::setTransform(nullptr);

				bgfx::frame();
				return true;
			}

			return false;
		}
		void DrawAllVoxelSquere(const uint64_t& state)
		{
			auto&& instance = SpanData::getInstance();
			auto&& Data = instance.Data;
			for (int i = 0; i < instance.Data.size(); i++)
			{
				SpanList& List = instance.Data[i];
				int sphereIndex = 0;
				using namespace segment_mgr;

				const Tile& tile = Spheres[sphereIndex]->GetTileByIndex(List.TileIndex);
				for (int j = 0; j < List.Spans.size(); j++)
				{
					glm::vec3 axis_y = glm::vec3(0,1,0);

					glm::vec3 SpanworldPos = List.CenteralWorldPos + (axis_y * (List.Spans[j].bottom + List.Spans[j].top) / 2.0f);
					float y_scale = (std::abs(List.Spans[j].top - List.Spans[j].bottom));

					glm::mat4 matrix = glm::translate(glm::mat4(1.0f), SpanworldPos) * glm::scale(glm::mat4(1.0f), glm::vec3(0.5 * 32.0f, 0.5 * y_scale, 0.5 * 32.0f));

					bgfx::setTransform(glm::value_ptr(matrix));
					bgfx::setVertexBuffer(0, CubeVertexBuffer);
					bgfx::setIndexBuffer(CubeIndicesBuffer);
					bgfx::setState(state);
					bgfx::submit(0, m_program);
					bgfx::setTransform(nullptr);
				}
			}
		}
		void DrawAllVoxelCylinder(const uint64_t& state)
		{
			auto&& instance = SpanData::getInstance();
			auto&& Data = instance.Data;
			for (int i = 0; i < instance.Data.size(); i++)
			{
				SpanList& List = instance.Data[i];
				int sphereIndex = 0;
				using namespace segment_mgr;

				const Tile& tile = Spheres[sphereIndex]->GetTileByIndex(List.TileIndex);
				for (int j = 0; j < List.Spans.size(); j++)
				{
					glm::vec3 axis_y = -glm::normalize(glm::vec3(tile.centerPos.x,0, tile.centerPos.z));

					glm::mat4 rotationMatrix = glm::mat4(glm::vec4(tile.axis_u, 0), glm::vec4(axis_y, 0), glm::vec4(tile.axis_v, 0), glm::vec4(0, 0, 0, 1));

					glm::vec3 SpanworldPos = List.CenteralWorldPos + (axis_y * (List.Spans[j].bottom + List.Spans[j].top) / 2.0f);
					float y_scale = (std::abs(List.Spans[j].top - List.Spans[j].bottom));

					glm::mat4 matrix = glm::translate(glm::mat4(1.0f), SpanworldPos) *
						rotationMatrix * glm::scale(glm::mat4(1.0f), glm::vec3(0.5 * 32.0f, 0.5 * y_scale, 0.5 * 32.0f));

					bgfx::setTransform(glm::value_ptr(matrix));
					bgfx::setVertexBuffer(0, CubeVertexBuffer);
					bgfx::setIndexBuffer(CubeIndicesBuffer);
					bgfx::setState(state);
					bgfx::submit(0, m_program);
					bgfx::setTransform(nullptr);
				}
			}
		}

		void DrawAllVoxel(const uint64_t& state)
		{
			auto&& instance = SpanData::getInstance();
			auto&& Data = instance.Data;
			for (int i = 0; i < instance.Data.size(); i++)
			{
				SpanList& List = instance.Data[i];
				int sphereIndex = 0;
				using namespace segment_mgr;

				const Tile& tile = Spheres[sphereIndex]->GetTileByIndex(List.TileIndex);
				for (int j = 0; j < List.Spans.size(); j++)
				{
					glm::vec3 axis_y = glm::normalize(tile.centerPos);// (1,0,0)

					glm::mat4 rotationMatrix = glm::mat4(glm::vec4(tile.axis_u, 0), glm::vec4(axis_y, 0), glm::vec4(tile.axis_v, 0), glm::vec4(0, 0, 0, 1));

					glm::vec3 SpanworldPos = List.CenteralWorldPos + (axis_y * (List.Spans[j].bottom + List.Spans[j].top) / 2.0f);
					float y_scale = (std::abs(List.Spans[j].top - List.Spans[j].bottom));

					glm::mat4 matrix = glm::translate(glm::mat4(1.0f), SpanworldPos) *
						rotationMatrix * glm::scale(glm::mat4(1.0f), glm::vec3(0.5 * 32.0f, 0.5 * y_scale, 0.5 * 32.0f));

					bgfx::setTransform(glm::value_ptr(matrix));
					bgfx::setVertexBuffer(0, CubeVertexBuffer);
					bgfx::setIndexBuffer(CubeIndicesBuffer);
					bgfx::setState(state);
					bgfx::submit(0, m_program);
					bgfx::setTransform(nullptr);
				}
			}
		}
		void DrawWays(const uint64_t& state)
		{
			
			
			DebugDrawEncoder dde;
			dde.begin(0);
			auto&& instance = SpanData::getInstance();
			if (way.size() < 1)
				return;

			for (int i = 0; i < way.size() - 1; i++)
			{
				dde.push();
				{
					const segment_mgr::Tile& fromTile = Spheres[0]->GetTileByIndex(instance.Data[way[i]->sp->ListIndex].TileIndex);
					const segment_mgr::Tile& toTile =   Spheres[0]->GetTileByIndex(instance.Data[way[i + 1]->sp->ListIndex].TileIndex);
					glm::vec3 axis_y1 = glm::normalize(glm::cross(fromTile.axis_u, fromTile.axis_v));
					glm::vec3 axis_y2 = glm::normalize(glm::cross(toTile.axis_u, toTile.axis_v));

					glm::vec3 from = instance.Data[way[i]->sp->ListIndex].CenteralWorldPos + (axis_y1 * way[i]->sp->top);
					glm::vec3 to = instance.Data[way[i+1]->sp->ListIndex].CenteralWorldPos + (axis_y2 * way[i+1]->sp->top);
					float radius = 2.0f;
					dde.drawCylinder({ from.x,from.y,from.z }, { to.x,to.y,to.z }, radius);
				}
				dde.pop();
			}

			dde.end();
			
			
		}
		entry::MouseState m_mouseState;

		uint32_t m_width;
		uint32_t m_height;
		uint32_t m_debug;
		uint32_t m_reset;
		bgfx::VertexBufferHandle CubeVertexBuffer;
		bgfx::IndexBufferHandle CubeIndicesBuffer;

		bgfx::ProgramHandle m_program;
		bgfx::ProgramHandle m_program_instanced;

		int64_t m_timeOffset;

		std::unordered_map<std::string, std::shared_ptr<RenderMesh>> Meshes;
		float ExampleCubes::cameraPosNow[3];

		uint32_t m_reading;
		uint32_t m_currFrame;

		glm::vec3 fromPos = {0,0,0};
		glm::vec3 toPos = {0,0,0};

		std::vector<std::shared_ptr<voxelFuncs::wayNode>> way;

	private:
		std::unique_ptr<SceneMgr> SceneMgrPtr;
		std::vector<std::shared_ptr<segment_mgr::SegmentMgr>> Spheres;
	};


} // namespace

ENTRY_IMPLEMENT_MAIN(
	ExampleCubes
	, "01-cubes"
	, "Rendering simple static mesh."
	, "https://bkaradzic.github.io/bgfx/examples.html#cubes"
);






