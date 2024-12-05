/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "denso_robot_core/denso_controller_rc8_cobotta.h"

namespace denso_robot_core {

// Minimum Cobotta version for
// "ExecResetStoState" and "ExecManualResetPreparation" functions
Version MIN_VERSION("2.8.0");

HRESULT DensoControllerRC8Cobotta::AddRobot(XMLElement * xmlElem)
{
  std::size_t objs;
  HRESULT hr;
  Name_Vec vecName;

  hr = DensoBase::GetObjectNames(ID_CONTROLLER_GETROBOTNAMES, vecName);
  if (SUCCEEDED(hr)) {
    for (objs = 0; objs < vecName.size(); objs++) {
      Handle_Vec vecHandle;
      hr = DensoBase::AddObject(ID_CONTROLLER_GETROBOT, vecName[objs], vecHandle);
      if (FAILED(hr)) {
        break;
      }

      DensoRobot_Ptr rob(
        new DensoRobotRC8Cobotta(m_node, this, m_vecService, vecHandle, vecName[objs], m_mode));
      hr = rob->InitializeBCAP(xmlElem);
      if (FAILED(hr)) {
        break;
      }

      m_vecRobot.push_back(rob);
    }
  }

  return hr;
}

HRESULT DensoControllerRC8Cobotta::get_Robot(int index, DensoRobotRC8Cobotta_Ptr * robot)
{
  if (robot == NULL) {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), m_vecRobot.begin(), m_vecRobot.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, index, &pBase);
  if (SUCCEEDED(hr)) {
    *robot = std::dynamic_pointer_cast<DensoRobotRC8Cobotta>(pBase);
  }

  return hr;
}

/**
 * Clear an error that occurs in the controller.
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoControllerRC8Cobotta::ExecClearError()
{
  DensoRobotRC8Cobotta_Ptr pRob;
  HRESULT hr;

  Version currVersion(m_ctrlVer);

  if (MIN_VERSION > currVersion) {
    return S_OK;
  }

  hr = this->get_Robot(0, &pRob);
  if (FAILED(hr)) {
    return hr;
  }

  hr = pRob->ExecManualResetPreparation();
  if (FAILED(hr)) {
    return hr;
  }

  hr = DensoControllerRC8::ExecClearError();
  if (FAILED(hr)) {
    return hr;
  }

  return hr;
}

/**
 * Reset the STO(Safe Torque Off) state.
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoControllerRC8Cobotta::ExecResetStoState()
{
  DensoRobotRC8Cobotta_Ptr pRob;
  HRESULT hr;

  Version currVersion(m_ctrlVer);

  if (MIN_VERSION > currVersion) {
    return S_OK;
  }

  hr = this->get_Robot(0, &pRob);
  if (FAILED(hr)) {
    return hr;
  }
  hr = pRob->ExecManualResetPreparation();
  if (FAILED(hr)) {
    return hr;
  }
  hr = pRob->ExecMotionPreparation();
  if (FAILED(hr)) {
    return hr;
  }

  return hr;
}

/**
 * @param robot_name
 * @return True if robot_name is COBOTTA.
 */
bool DensoControllerRC8Cobotta::IsCobotta(const std::string& robot_name)
{
  std::string cobotta = ROBOT_NAME_RC8_COBOTTA;

  if (
    std::equal(
      cobotta.begin(),cobotta.end(), robot_name.begin(), robot_name.begin() + cobotta.length()))
  {
    return true;
  }

  return false;
}

HRESULT DensoControllerRC8Cobotta::HandMove(const double w)
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT *pvnt;
  VARIANT_Ptr vntRet(new VARIANT());
  for (argc = 0; argc < BCAP_CONTROLLER_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_I4;
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_WATCH];
        break;

      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"HandMoveA");
        break;

      case 2:
        vntTmp->vt = (VT_ARRAY | VT_VARIANT);
        if (w < 0) {
          vntTmp->parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
          SafeArrayAccessData(vntTmp->parray, (void**)&pvnt);

          pvnt[0].vt = VT_R8;
          pvnt[0].dblVal = 30.0;

          pvnt[1].vt = VT_UI1;
          pvnt[1].bVal = 100;
        } else {
          vntTmp->parray = SafeArrayCreateVector(VT_VARIANT, 0, 3);
          SafeArrayAccessData(vntTmp->parray, (void**)&pvnt);

          pvnt[0].vt = VT_R8;
          pvnt[0].dblVal = w;

          pvnt[1].vt = VT_UI1;
          pvnt[1].bVal = 100;

          //pvnt[2].vt = VT_R8;
          //pvnt[2].dblVal = 20.0;

          pvnt[2].vt = VT_BSTR;
          pvnt[2].bstrVal = SysAllocString(L"Next");
          SafeArrayUnaccessData(vntTmp->parray);
        }
        break;
    }
    vntArgs.push_back(*vntTmp.get());
  }
  return m_vecService[DensoBase::SRV_WATCH]->ExecFunction(ID_CONTROLLER_EXECUTE, vntArgs, vntRet);
}
}  // namespace denso_robot_core
