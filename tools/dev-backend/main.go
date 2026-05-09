package main

import (
	"log"
	"net/http"
	"os/exec"
	"path/filepath"
)

func main() {
	http.HandleFunc("/analyze", func(w http.ResponseWriter, r *http.Request) {
		file := r.URL.Query().Get("file")
		if file == "" {
			http.Error(w, `{"error":"missing file param"}`, http.StatusBadRequest)
			return
		}
		abs, err := filepath.Abs(file)
		if err != nil {
			http.Error(w, `{"error":"invalid path"}`, http.StatusBadRequest)
			return
		}
		args := []string{}
		if r.URL.Query().Get("thumbnails") == "1" {
			args = append(args, "--thumbnails")
		}
		if start := r.URL.Query().Get("start"); start != "" {
			count := r.URL.Query().Get("count")
			if count == "" {
				count = "50"
			}
			args = append(args, "--range", start, count)
		}
		args = append(args, abs)
		out, err := exec.Command("../../analyzer-core/stream-analyzer-core", args...).Output()
		if err != nil {
			http.Error(w, `{"error":"analyzer failed: `+err.Error()+`"}`, http.StatusInternalServerError)
			return
		}
		w.Header().Set("Content-Type", "application/json")
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.Write(out)
	})

	log.Println("Backend listening on :9210")
	log.Fatal(http.ListenAndServe(":9210", nil))
}
