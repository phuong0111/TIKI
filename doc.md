# Tài liệu hệ thống tối ưu định tuyến xe vận chuyển container

## 1. Tổng quan hệ thống

### 1.1 Mục đích
Hệ thống được thiết kế để giải quyết bài toán tối ưu hóa định tuyến xe tải vận chuyển container với các đặc điểm sau:
- Có nhiều xe tải xuất phát từ các depot khác nhau
- Mỗi xe có thể thực hiện nhiều request vận chuyển container
- Có điểm đặt rơ-mooc (trailer) cố định
- Container có thể được vận chuyển theo 2 cách:
  + Dùng rơ-mooc: Xe phải đi lấy rơ-mooc trước khi lấy container
  + Không dùng rơ-mooc: Xe có thể trực tiếp lấy container

### 1.2 Cấu trúc dữ liệu chính

#### Enum Action
```cpp
enum Action {
    PICKUP_CONTAINER,           // Lấy container (cần rơ-mooc)
    PICKUP_CONTAINER_TRAILER,   // Lấy container (đã có rơ-mooc)
    DROP_CONTAINER,            // Trả container (giữ rơ-mooc)
    DROP_CONTAINER_TRAILER,    // Trả container (bỏ rơ-mooc)
    PICKUP_TRAILER,           // Lấy rơ-mooc
    DROP_TRAILER,            // Trả rơ-mooc
    STOP                    // Kết thúc lộ trình
}
```

#### Struct Request
```cpp
struct Request {
    int id;                    // ID của request
    ContainerSize size;        // Kích thước container (20/40 feet)
    int pickup_point;          // Điểm lấy hàng
    Action pickup_action;      // Hành động khi lấy
    ll pickup_duration;        // Thời gian xử lý tại điểm lấy
    int drop_point;           // Điểm trả hàng
    Action drop_action;       // Hành động khi trả
    ll drop_duration;        // Thời gian xử lý tại điểm trả
}
```

#### Struct Route
```cpp
struct Route {
    int depot;                // Điểm xuất phát
    std::list<int> list_reqs; // Danh sách các request
    ll cost;                  // Tổng chi phí của lộ trình
}
```

#### Struct RequestContext
```cpp
struct RequestContext {
    int request_id;           // ID của request
    ll transitionCost;        // Chi phí chuyển tiếp (từ điểm trước -> hiện tại -> điểm sau)
    ll selfCost;             // Chi phí của bản thân request (pickup + drop)
    int routeIdx;            // Index của route chứa request
    std::list<int>::iterator position;  // Vị trí trong route
}
```

## 2. Các thành phần chính của hệ thống

### 2.1 Tính toán chi phí (Cost Calculation)

#### calculateRequestContextCost
```cpp
ll calculateRequestContextCost(const Request &req)
```
- **Mục đích**: Tính tổng chi phí của một request độc lập
- **Công thức**: Thời gian xử lý tại điểm lấy + Thời gian xử lý tại điểm trả + Khoảng cách di chuyển giữa 2 điểm

#### calculateDepotToRequestCost
```cpp
ll calculateDepotToRequestCost(const int &depot, const Request &req)
```
- **Mục đích**: Tính chi phí từ depot đến điểm lấy hàng đầu tiên
- **Xử lý đặc biệt**: 
  + Nếu cần rơ-mooc (PICKUP_CONTAINER): Chi phí = Depot -> Điểm rơ-mooc + Thời gian lấy rơ-mooc + Điểm rơ-mooc -> Điểm lấy
  + Nếu không cần (PICKUP_CONTAINER_TRAILER): Chi phí = Depot -> Điểm lấy

#### calculateRequestToDepotCost
```cpp
ll calculateRequestToDepotCost(const Request &req, const int &depot)
```
- **Mục đích**: Tính chi phí từ điểm trả hàng cuối về depot
- **Xử lý đặc biệt**:
  + Nếu còn rơ-mooc (DROP_CONTAINER): Chi phí = Điểm trả -> Điểm rơ-mooc + Thời gian trả rơ-mooc + Điểm rơ-mooc -> Depot
  + Nếu không còn (DROP_CONTAINER_TRAILER): Chi phí = Điểm trả -> Depot

#### calculateRequestTransitionCost
```cpp
ll calculateRequestTransitionCost(const Request &curr_req, const Request &next_req)
```
- **Mục đích**: Tính chi phí chuyển tiếp giữa 2 request liên tiếp
- **Các trường hợp**:
  1. DROP_CONTAINER -> PICKUP_CONTAINER_TRAILER:
     - Cần trả rơ-mooc tại điểm rơ-mooc
     - Chi phí = Điểm trả hiện tại -> Điểm rơ-mooc + Thời gian xử lý + Điểm rơ-mooc -> Điểm lấy tiếp theo
  2. DROP_CONTAINER_TRAILER -> PICKUP_CONTAINER:
     - Cần lấy rơ-mooc tại điểm rơ-mooc
     - Chi phí tương tự case 1
  3. Trường hợp thông thường:
     - Chi phí = Khoảng cách trực tiếp giữa điểm trả hiện tại và điểm lấy tiếp theo

### 2.2 Quản lý Request Context

#### updateRequestContext
```cpp
void updateRequestContext(int req_id, Route &route, std::list<int>::iterator it)
```
- **Mục đích**: Cập nhật thông tin context của một request
- **Các thông tin cập nhật**:
  + request_id: ID của request
  + routeIdx: Index của route chứa request
  + position: Iterator trỏ tới vị trí của request trong route
  + selfCost: Chi phí của bản thân request
  + transitionCost: Chi phí chuyển tiếp với các request xung quanh
- **Các trường hợp tính transitionCost**:
  1. Request đầu tiên trong route:
     - Tính từ depot đến request này
     - Tính từ request này đến request tiếp theo (nếu có)
     - Trừ đi chi phí trực tiếp từ depot đến request tiếp theo
  2. Request cuối cùng trong route:
     - Tính từ request trước đó đến request này
     - Tính từ request này về depot
     - Trừ đi chi phí trực tiếp từ request trước về depot
  3. Request ở giữa:
     - Tính chi phí từ request trước -> hiện tại -> request sau
     - Trừ đi chi phí trực tiếp từ request trước đến request sau

### 2.3 Các thao tác với Request

#### removeStopsByRequestId
```cpp
void removeStopsByRequestId(Route &route, int request_id)
```
- **Mục đích**: Xóa một request khỏi route
- **Quy trình**:
  1. Tính chi phí mới của route khi bỏ request (calculateRemovalCost)
  2. Xóa request khỏi danh sách
  3. Cập nhật chi phí mới cho route

#### insertRequests
```cpp
void insertRequests(const std::vector<int> &requestIds)
```
- **Mục đích**: Chèn nhiều request vào các route
- **Quy trình**:
  1. Xáo trộn ngẫu nhiên thứ tự các request
  2. Với mỗi request:
     - Thử chèn vào tất cả các vị trí có thể trong mọi route
     - Tính chi phí cho mỗi vị trí (calculateInsertionCost)
     - Chọn vị trí có chi phí thấp nhất
     - Thực hiện chèn và cập nhật context

### 2.4 Giải thuật tối ưu (Simulated Annealing)

#### solve
```cpp
void solve()
```
- **Mục đích**: Tìm lời giải tối ưu cho bài toán
- **Tham số điều chỉnh**:
  + temperature: Nhiệt độ ban đầu (100.0)
  + coolingRate: Tốc độ làm lạnh (0.9995)
  + alpha: Trọng số cho chi phí tối đa (F1)
- **Quy trình**:
  1. Khởi tạo lời giải ban đầu bằng cách chèn tuần tự các request
  2. Lặp cho đến khi:
     - Đạt số vòng lặp tối đa HOẶC
     - Nhiệt độ quá thấp (< 1e-8) HOẶC
     - Hết thời gian cho phép (29.5s)
  3. Trong mỗi vòng lặp:
     - Chọn ngẫu nhiên số lượng request cần xóa
     - Xóa các request có chi phí cao (removeRandomRequests)
     - Chèn lại các request đã xóa (insertRequests)
     - So sánh chi phí mới với chi phí tốt nhất
     - Chấp nhận lời giải xấu hơn với xác suất phụ thuộc vào nhiệt độ
     - Giảm nhiệt độ (currentTemp *= coolingRate)

### 2.5 Hàm mục tiêu

#### calculateSolutionCost
```cpp
ll calculateSolutionCost()
```
- **Mục đích**: Tính chi phí tổng thể của lời giải
- **Công thức**: alpha * F1 + F2
  + F1 = max(route_cost): Chi phí của route đắt nhất
  + F2 = sum(route_cost): Tổng chi phí của tất cả route
- **Ý nghĩa**:
  + F1 đảm bảo cân bằng tải giữa các xe
  + F2 tối thiểu hóa tổng chi phí toàn hệ thống
  + alpha điều chỉnh mức độ ưu tiên giữa cân bằng tải và tổng chi phí

## 3. Tối ưu hiệu năng

### 3.1 Sử dụng global RequestContext
- Tránh tính toán lại các chi phí không thay đổi
- Truy cập nhanh O(1) thông tin request
- Dễ dàng cập nhật khi cấu trúc route thay đổi

### 3.2 Tính toán chi phí járementally
- calculateInsertionCost: Chỉ tính chi phí thay đổi khi chèn
- calculateRemovalCost: Chỉ tính chi phí thay đổi khi xóa
- Tránh tính lại toàn bộ chi phí route

### 3.3 Quản lý iterator hợp lý
- Lưu trữ và cập nhật iterator trong RequestContext
- Xử lý cẩn thận khi iterator bị invalid sau thao tác xóa
- Cập nhật context cho các request bị ảnh hưởng

## 4. Cách sử dụng

### 4.1 Input Format
```
POINTS {số điểm}
DISTANCES {số lượng khoảng cách}
{from} {to} {distance} // Ma trận khoảng cách
TRAILER {điểm đặt rơ-mooc} {thời gian xử lý rơ-mooc}
VEHICLES {số lượng xe}
{id xe} {depot} // Thông tin từng xe
REQUEST
{id} {size} {pickup_point} {pickup_action} {pickup_duration} {drop_point} {drop_action} {drop_duration}
#
```

### 4.2 Output Format
```
ROUTES {số lượng xe}
TRUCK {id xe}
{point} {action} [{request_id}]
#
```

### 4.3 Tham số điều chỉnh chính
1. **alpha** (trọng số F1):
   - Giá trị lớn: Ưu tiên cân bằng tải
   - Giá trị nhỏ: Ưu tiên tổng chi phí
   
2. **temperature và coolingRate**:
   - temperature cao + coolingRate gần 1: Tìm kiếm rộng hơn
   - temperature thấp + coolingRate nhỏ: Hội tụ nhanh hơn

3. **max_attempt** (số request xóa mỗi lần):
   - Giá trị lớn: Thay đổi nhiều hơn, có thể thoát local optimal
   - Giá trị nhỏ: Thay đổi ít hơn, hội tụ nhanh hơn

## 5. Ví dụ minh họa

### 5.1 Kịch bản đơn giản
```cpp
// Input
POINTS 5
DISTANCES 25
0 1 10
1 0 10
// ... ma trận khoảng cách
TRAILER 2 30
VEHICLES 2
1 0
2 1
REQUEST
1 20 3 PICKUP_CONTAINER 20 4 DROP_CONTAINER 20
2 40 1 PICKUP_CONTAINER_TRAILER 30 0 DROP_CONTAINER_TRAILER 30
#

// Output có thể
ROUTES 2
TRUCK 1
2 PICKUP_TRAILER
3 PICKUP_CONTAINER 1
4 DROP_CONTAINER 1
2 DROP_TRAILER
0 STOP
#
TRUCK 2
1 PICKUP_CONTAINER_TRAILER 2
0 DROP_CONTAINER_TRAILER 2
1 STOP
#
```

## 5. Ví dụ minh họa (tiếp)

### 5.2 Giải thích
1. Request 1:
   - Cần rơ-mooc (PICKUP_CONTAINER)
   - Quy trình: Depot -> Lấy rơ-mooc (2) -> Lấy container (3) -> Trả container (4) -> Trả rơ-mooc (2) -> Depot
   - Chi phí = Chi phí di chuyển + Thời gian xử lý rơ-mooc + Thời gian xử lý container

2. Request 2:
   - Đã có rơ-mooc (PICKUP_CONTAINER_TRAILER)
   - Quy trình đơn giản: Depot -> Lấy container (1) -> Trả container (0) -> Depot
   - Chi phí = Chi phí di chuyển + Thời gian xử lý container

### 5.3 Các trường hợp đặc biệt

1. **Chuyển tiếp giữa các request**:
```plaintext
Request A (DROP_CONTAINER) -> Request B (PICKUP_CONTAINER_TRAILER):
- Xe đang có rơ-mooc sau khi trả container A
- Cần trả rơ-mooc trước khi thực hiện request B
- Quy trình: Drop A -> Trả rơ-mooc -> Lấy B
```

2. **Tối ưu sử dụng rơ-mooc**:
```plaintext
Request A (DROP_CONTAINER) -> Request B (PICKUP_CONTAINER):
- Giữ rơ-mooc sau request A
- Tiếp tục dùng cho request B
- Tiết kiệm chi phí lấy/trả rơ-mooc
```

3. **Cân bằng tải giữa các xe**:
```plaintext
Xe 1: Request có pickup/drop xa depot
Xe 2: Request có pickup/drop gần depot
-> Phân bổ để cân bằng chi phí tổng
```

## 6. Xử lý các tình huống đặc biệt

### 6.1 Invalid Iterator sau khi xóa

**Vấn đề**:
```cpp
// Lỗi: Iterator không còn hợp lệ sau khi xóa
auto it = requestContexts[selectedRequestId].position;
removeStopsByRequestId(route, selectedRequestId);
auto nextIt = std::next(it);  // BUG: it đã invalid
```

**Giải pháp**:
```cpp
// Lưu ID trước khi xóa
auto currentPos = requestContexts[selectedRequestId].position;
int prevRequestId = -1, nextRequestId = -1;
if (currentPos != route.list_reqs.begin()) {
    prevRequestId = *std::prev(currentPos);
}
if (std::next(currentPos) != route.list_reqs.end()) {
    nextRequestId = *std::next(currentPos);
}

// Xóa request
removeStopsByRequestId(route, selectedRequestId);

// Cập nhật context bằng ID
if (prevRequestId != -1) {
    auto it = std::find(route.list_reqs.begin(), route.list_reqs.end(), prevRequestId);
    updateRequestContext(prevRequestId, route, it);
}
```

### 6.2 Cập nhật chi phí route

**Phương pháp 1: Tính toàn bộ**
```cpp
ll calculateRouteCost(const Route &route) {
    // Tính lại toàn bộ chi phí
    // Chậm nhưng chính xác
    // Dùng để verify trong debug
}
```

**Phương pháp 2: Tính járemental**
```cpp
ll calculateInsertionCost(const Route &route, const int request_id, std::list<int>::iterator position) {
    // Chỉ tính chi phí thay đổi
    // Nhanh hơn nhiều
    // Dùng trong quá trình tối ưu
}
```

### 6.3 Xử lý ràng buộc thời gian

```cpp
void solve() {
    auto start_time = chrono::high_resolution_clock::now();
    
    while (/* điều kiện khác */) {
        auto current_time = chrono::high_resolution_clock::now();
        double elapsed_time = chrono::duration_cast<chrono::milliseconds>
            (current_time - start_time).count() / 1000.0;
            
        if (elapsed_time >= 29.50) break;  // Dừng sớm để đảm bảo output
        
        // Xử lý tối ưu
    }
}
```

## 7. Debug và Kiểm thử

### 7.1 Verify tính đúng đắn của lời giải

```cpp
bool verifyRoute(const Route &route) {
    // Kiểm tra tính hợp lệ của route
    bool has_trailer = false;
    int prev_point = route.depot;
    
    for (const int req_id : route.list_reqs) {
        const Request &req = requests[req_id];
        
        // Kiểm tra điều kiện rơ-mooc
        if (req.pickup_action == PICKUP_CONTAINER && !has_trailer) {
            // Error: Cần rơ-mooc nhưng không có
            return false;
        }
        
        // Cập nhật trạng thái
        has_trailer = (req.drop_action != DROP_CONTAINER_TRAILER);
        prev_point = req.drop_point;
    }
    
    return true;
}
```

### 7.2 So sánh chi phí

```cpp
void validateCosts() {
    for (const Route &route : currentSolution) {
        ll calculated = calculateRouteCost(route);
        ll stored = route.cost;
        
        if (calculated != stored) {
            std::cout << "Cost mismatch in route!" << std::endl;
            std::cout << "Stored: " << stored << std::endl;
            std::cout << "Calculated: " << calculated << std::endl;
        }
    }
}
```

### 7.3 Log chi tiết

```cpp
void logRouteDetails(const Route &route, bool verbose = false) {
    if (!verbose) return;
    
    std::cout << "Route from depot " << route.depot << std::endl;
    std::cout << "Requests: ";
    for (int req_id : route.list_reqs) {
        std::cout << req_id << " ";
    }
    std::cout << "\nCost: " << route.cost << std::endl;
    std::cout << "------------------------" << std::endl;
}
```

## 8. Tối ưu thêm

### 8.1 Cải thiện hiệu năng

1. **Cache khoảng cách**:
```cpp
// Thay vì tính lại nhiều lần
std::unordered_map<std::pair<int,int>, ll> distanceCache;

ll getCachedDistance(int from, int to) {
    auto key = std::make_pair(from, to);
    if (distanceCache.count(key) == 0) {
        distanceCache[key] = calculateDistance(from, to);
    }
    return distanceCache[key];
}
```

2. **Tối ưu removeRandomRequests**:
```cpp
// Sử dụng min-heap thay vì sort
std::priority_queue<std::pair<ll, int>> costHeap;
for (auto it = route.list_reqs.begin(); it != route.list_reqs.end(); ++it) {
    int req_id = *it;
    costHeap.push({requestContexts[req_id].transitionCost, req_id});
}
```

3. **Tối ưu insertRequests**:
```cpp
// Thử insert vào các vị trí có tiềm năng trước
bool isPromising(const Route &route, const Request &req) {
    return route.cost < averageRouteCost;
}
```

### 8.2 Cải thiện chất lượng giải

1. **Điều chỉnh nhiệt độ động**:
```cpp
double adaptiveTemperature = temperature * 
    (1.0 + log(1.0 + currentSolutionCost/initialSolutionCost));
```

2. **Thay đổi chiến lược chọn request**:
```cpp
// Ưu tiên request có ảnh hưởng lớn đến balance
double getRequestImbalanceImpact(int req_id) {
    Route &route = currentSolution[requestContexts[req_id].routeIdx];
    return abs(route.cost - averageRouteCost);
}
```

3. **Local search sau khi tìm được giải pháp tốt**:
```cpp
void localSearch() {
    bool improved;
    do {
        improved = false;
        // Thử hoán đổi các cặp request
        // Thử di chuyển single request
        // Update improved nếu tìm được giải pháp tốt hơn
    } while (improved);
}
```
