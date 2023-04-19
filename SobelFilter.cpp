#include <cmath>
#include <iomanip>

#include "SobelFilter.h"
using namespace std;

SobelFilter::SobelFilter(sc_module_name n)
    : sc_module(n), t_skt("t_skt"), base_offset(0)
{
  SC_THREAD(do_filter);

  t_skt.register_b_transport(this, &SobelFilter::blocking_transport);
}

SobelFilter::~SobelFilter() = default;

void SobelFilter::do_filter()
{
  int count = 0;
  while (true)
  {
    unsigned int a[9], flag = 0;
    for (int j = 0; j < 9; j++)
    {
      // 從input的FIFO通道中讀取像素，添加到buffer
      int pixel = (i_r.read() + i_g.read() + i_b.read()) / 3;
      buffer.push_back(pixel);
      count++;
      cout << "count:" << count << endl;
    }
    // 如果buffer中的像素數量達到所需数量，則排序然後計算中值
    if (buffer.size() == 9)
    {

      for (int i = 0; i < 9; i++)
      {
        a[i] = buffer[i];
      }
      flag = 1; // 代表buffer的data已經傳輸完畢

      std::sort(buffer.begin(), buffer.end()); // sorting
      a[4] = buffer[4];

      // 刪除buffer裡的資料
      buffer.clear();
    }

    if (flag == 1)
    {
      int total = 0, k = 0; // Mean filter
      for (unsigned int v = 0; v < MASK_Y; ++v)
      {
        for (unsigned int u = 0; u < MASK_X; ++u)
        {
          total += a[k] * mask[v][u];
          k++;
        }
      }
      int result = (total / 9);
      o_result.write(result);
      // wait(10); // emulate module delay
    }
  }
}

void SobelFilter::blocking_transport(tlm::tlm_generic_payload &payload,
                                     sc_core::sc_time &delay)
{
  sc_dt::uint64 addr = payload.get_address();
  addr -= base_offset;
  unsigned char *mask_ptr = payload.get_byte_enable_ptr();
  unsigned char *data_ptr = payload.get_data_ptr();
  word buffer;
  switch (payload.get_command())
  {
  case tlm::TLM_READ_COMMAND:
    switch (addr)
    {
    case SOBEL_FILTER_RESULT_ADDR:
      buffer.uint = o_result.read();
      break;
    case SOBEL_FILTER_CHECK_ADDR:
      buffer.uint = o_result.num_available();
      break;
    default:
      std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;
    }
    data_ptr[0] = buffer.uc[0];
    data_ptr[1] = buffer.uc[1];
    data_ptr[2] = buffer.uc[2];
    data_ptr[3] = buffer.uc[3];
    break;
  case tlm::TLM_WRITE_COMMAND:
    switch (addr)
    {
    case SOBEL_FILTER_R_ADDR:
      if (mask_ptr[0] == 0xff)
      {
        i_r.write(data_ptr[0]);
      }
      if (mask_ptr[1] == 0xff)
      {
        i_g.write(data_ptr[1]);
      }
      if (mask_ptr[2] == 0xff)
      {
        i_b.write(data_ptr[2]);
      }
      break;
    default:
      std::cerr << "Error! SobelFilter::blocking_transport: address 0x"
                << std::setfill('0') << std::setw(8) << std::hex << addr
                << std::dec << " is not valid" << std::endl;
    }
    break;
  case tlm::TLM_IGNORE_COMMAND:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  default:
    payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
    return;
  }
  payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
}