// Synthetic validation of the blue chroma-key rescue channel in
// psvr_camera.mm's process_frame mask stage. Mirrors the exact ops
// (0.5*(G+R) chroma, adaptive top-N threshold, union with luma). Not
// wired into cmake. Build + run:
//
//   clang++ -std=c++20 -O1 $(pkg-config --cflags --libs opencv4) \
//       chroma_test.cpp -o /tmp/chroma && /tmp/chroma
//
// Proves the regression-safety claim: a bright white interferer pins
// the LUMA threshold above a (dim) blue LED so luma alone misses it,
// but the chroma channel isolates the LED while suppressing the white,
// and the luma|chroma union still catches bloomed-white LED cores that
// chroma (B=G=R -> chroma 0) would miss.
// suppressed in chroma, bloomed-white LEDs still caught by luma union.
#include <opencv2/opencv.hpp>
#include <cstdio>
static int adaptive_topN(const cv::Mat& c, int target, int fl, int ce){
    int hs=256; float rg[]={0,256}; const float* rp=rg; cv::Mat h;
    cv::calcHist(&c,1,nullptr,cv::Mat(),h,1,&hs,&rp);
    int cum=0; for(int v=255;v>=0;--v){cum+=(int)h.at<float>(v);
        if(cum>=target) return std::max(fl,std::min(ce,v));} return fl;
}
int main(){
    // 100x100 dark frame. Regions:
    //  blue LED    at (10,10) 5x5  BGR(200,40,30)  - dim-ish, NOT white
    //  white window at (50,10) 20x20 BGR(250,250,250) - bright interferer
    //  bloomed LED at (10,50) 4x4  BGR(255,255,255) - saturated core
    cv::Mat bgr(100,100,CV_8UC3, cv::Scalar(8,6,6)); // dark room
    cv::rectangle(bgr,{10,10,5,5},  cv::Scalar(200,40,30),-1);
    cv::rectangle(bgr,{50,10,20,20},cv::Scalar(250,250,250),-1);
    cv::rectangle(bgr,{10,50,4,4},  cv::Scalar(255,255,255),-1);

    cv::Mat gray; cv::cvtColor(bgr,gray,cv::COLOR_BGR2GRAY);
    int bt = adaptive_topN(gray,40,180,250);
    cv::Mat luma; cv::threshold(gray,luma,bt,255,cv::THRESH_BINARY);

    cv::Mat ch[3]; cv::split(bgr,ch);
    cv::Mat gr,chroma,blue; cv::addWeighted(ch[1],0.5,ch[2],0.5,0,gr);
    cv::subtract(ch[0],gr,chroma);
    int ct = adaptive_topN(chroma,40,40,200);
    cv::threshold(chroma,blue,ct,255,cv::THRESH_BINARY);

    auto at=[&](const cv::Mat&m,int x,int y){return m.at<uint8_t>(y,x)>0;};
    int fails=0;
    auto chk=[&](bool ok,const char*w){printf("%s %s\n",ok?"PASS":"FAIL",w);if(!ok)fails++;};

    printf("luma thresh=%d  chroma thresh=%d\n", bt, ct);
    // The white window pins luma to ceiling; check whether the blue LED
    // (gray ~ 0.114*200+0.587*40+0.299*30 = 55) survives luma:
    printf("blue LED luma-detected=%d (gray~55, thresh=%d)\n", at(luma,12,12), bt);
    chk(at(blue,12,12),   "blue LED present in CHROMA mask");
    chk(!at(blue,55,15),  "white window SUPPRESSED in chroma mask");
    chk(at(luma,11,51),   "bloomed-white LED present in LUMA mask");
    // Union: blue LED must be in final mask even if luma missed it
    cv::Mat uni; cv::bitwise_or(luma,blue,uni);
    chk(at(uni,12,12),    "blue LED present in UNION (chroma rescue works)");
    chk(at(uni,11,51),    "bloomed LED present in UNION (luma path intact)");
    printf("\n%s (%d fail)\n", fails?"FAILED":"ALL PASS", fails);
    return fails?1:0;
}
