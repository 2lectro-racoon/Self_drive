#include "main.h"

std::atomic<bool> keepRunning{true};

int MODE = 0;
int state = 0;
double distance = 0;
std::mutex state_mutex;

bool gpt = 0;

using json = nlohmann::json;

int main() {

    // for chatGpt
    std::string api_key = "";
    std::string image_path = "frame.jpg";
    // pigpio init
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed!" << std::endl;
        return 1;
    }
    // set pinmode
    gpioSetMode(SIGNAL0, PI_OUTPUT);
    gpioSetMode(SIGNAL1, PI_OUTPUT);
    gpioSetMode(SIGNAL2, PI_OUTPUT);
    gpioSetMode(LEDL, PI_OUTPUT);
    gpioSetMode(LEDR, PI_OUTPUT);
    gpioSetMode(TRIGP, PI_OUTPUT);
    gpioSetMode(RETURN0, PI_INPUT);
    gpioSetMode(RETURN1, PI_INPUT);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // multi thread
    std::thread signalThread(signalProcessing);
    std::thread gptThread(process_request, api_key, image_path); 
    
    // just take input from user , for state
    while (keepRunning) {
        std::cout << "Select MODE, 0 is stop, 1 is drive, -1 is End program: " << std::endl;
        std::cin >> MODE;
        if (MODE == -1) {
                std::cout << "shutdown now" << std::endl;
                keepRunning = false;
                break;
            }
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state = MODE;
        }
    }
    // gpio close
    gpioTerminate();
    //wait threads stop
    gptThread.join();
    signalThread.join();
    std::cout << "ALL threads stopped." << std::endl;

    return 0;
}

// Communicate pico
void signalProcessing(){
    while(keepRunning){
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if(state == 0) {
                gpioWrite(SIGNAL2, LOW);
                gpioWrite(SIGNAL1, LOW);
                gpioWrite(SIGNAL0, LOW);
            }
            else if(state == 1) {
                gpioWrite(SIGNAL2, LOW);
                gpioWrite(SIGNAL1, LOW);
                gpioWrite(SIGNAL0, HIGH);
            }
        }
    }
}

// preprocessing image

std::string encode_image(const cv::Mat& frame) {
    std::vector<unsigned char> buffer;
    cv::imencode(".jpg", frame, buffer);
    return base64_encode(buffer.data(), buffer.size());
}

// Communicate chatGpt

void process_request(const std::string& api_key, const std::string& image_path) {

    cpr::Header headers = {
        {"Content-Type", "application/json"},
        {"Authorization", "Bearer " + api_key}
    };
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera" << std::endl;
        return;
    }    

    cv::Mat frame;
    uint8_t cnt = 0;
    uint8_t delay_count = 0;
    while(keepRunning){
        if(gpioRead(RETURN0)){
            gpt = 1;
        }
        cap >> frame;
        if(gpt == 1){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            cnt++;
            auto start = std::chrono::high_resolution_clock::now();
            
            if (frame.empty()) {
                std::cerr << "Error: Blank frame grabbed" << std::endl;
                break;
            }
            std::string base64_image = encode_image(frame);
            std::string filename = "frame" + std::to_string(cnt) + ".jpg";
            cv::imwrite(filename, frame);
            std::cout<<"save frame"<<std::endl;

            json payload = {
                {"model", "gpt-4o"},
                {"messages", {
                    {
                        {"role", "user"},
                        {"content", {
                            {{"type", "text"}, {"text", "You a self driving car's AI that identifies one of  3 road features. If you see a parking sign signified with a letter 'P', reply with '1'. If you see a tunnel sign, reply with '2'. If you see a traffic light, reply with the illuminated color in the format of 'red/yellow/green'. If you see none of these, reply with 0. Only reply when you are sure of what you see."}},
                            {{"type", "image_url"}, {"image_url", {{"url", "data:image/jpeg;base64," + base64_image}}}}
                        }}
                    }
                    
                }},
                {"max_tokens", 50}
            };
            if(delay_count >= 1){
                state = 1;
                delay_count = 0;
                continue;
            }
            // API 요청 보내기
            cpr::Response response = cpr::Post(
                cpr::Url{"https://api.openai.com/v1/chat/completions"},
                headers,
                cpr::Body{payload.dump()}
            );

            if (response.status_code == 0) {
                std::cerr << "Request failed: " << response.error.message << std::endl;
                continue;
            }


            if (response.status_code == 429) {
                if(cnt == 1){
                    state = 0;
                    gpt = 0;
                }
                else if(cnt >= 2){
                    state = 1;
                    gpt = 0;
                }
                int retry_after = 20;
                delay_count++;
                std::cerr << "Rate limit exceeded, retrying after " << retry_after << " seconds." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(retry_after));
                continue;
            }
            if (response.text.empty()) {
                std::cerr << "Error: Received empty response from API" << std::endl;
                continue;
            }

            try {
                auto json_response = json::parse(response.text);
                const std::string& message_content = json_response["choices"][0]["message"]["content"];

                std::cout << "Thread ID: " << std::this_thread::get_id() << " Response: " << message_content << std::endl;
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end - start;
                std::cout << "Thread ID: " << std::this_thread::get_id() << " Time taken: " << elapsed.count() << " seconds" << std::endl;

                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    if (message_content == "red" || message_content == "Red") {
                        if(elapsed.count() <= 2){
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                        }
                        if(cnt >= 2){
                            state = 1;
                            gpt = 0;
                        }
                        else{
                            state = 0;
                        }
                        std::cout << "red sign" << std::endl;
                    }
                    else if (message_content == "green" || message_content == "Green") {
                        state = 1;
                        gpt = 0;
                        std::cout << "green sign" << std::endl;
                    }
                    else {
                        gpt = 0;
                        cnt = 0;
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                        
                    }
                }
            } catch (const json::parse_error& e) {
                std::cerr << "JSON parse error: " << e.what() << std::endl;
                std::cerr << "Response body: " << response.text << std::endl;
            }


            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
    cap.release();
}